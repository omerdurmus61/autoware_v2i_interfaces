#include "v2i_spat_rviz_panel/spat_panel.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include <QGridLayout>
#include <QGroupBox>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace v2i_spat_rviz_panel
{

namespace
{
constexpr double kWrapWindowDeciseconds = 600.0;
constexpr int kStaleTimeoutMs = 1500;

struct ConfiguredSignal
{
  int id;
  const char * name;
  uint16_t intersection_id;
  uint8_t signal_group;
};

const std::vector<ConfiguredSignal> kSignals{
  {227621, "MLK - Market left turn", 22762, 1},
  {1984613, "MLK - Georgia East", 19846, 13},
  {127532, "MLK - Lyndsey", 12753, 2},
  {241872, "MLK - Houston", 24187, 2},
  {173422, "MLK - Douglas", 17342, 2},
  {148672, "MLK - Peeples", 14867, 2},
  {198462, "MLK - Georgia West", 19846, 2},
  {227624, "MLK - Market West", 22762, 4},
};
}

SpatPanel::SpatPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  createUi();
}

void SpatPanel::createUi()
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  topic_label_ = new QLabel(QString("SPAT topic: %1").arg(QString::fromStdString(topic_)), this);
  topic_label_->setStyleSheet("color: #94a3b8; font-size: 11px;");
  layout->addWidget(topic_label_);

  auto * grid = new QGridLayout();
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(6);

  for (std::size_t index = 0; index < kSignals.size(); ++index) {
    const auto & signal = kSignals[index];
    auto * group = new QGroupBox(signal.name, this);
    group->setMinimumWidth(150);
    group->setStyleSheet(
      "QGroupBox { border: 1px solid #334155; border-radius: 6px; margin-top: 8px; "
      "padding: 8px; color: #000000; font-weight: bold; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 3px; }"
    );

    auto * card_layout = new QVBoxLayout(group);
    auto * state = new QLabel("STALE", group);
    auto * remaining = new QLabel("--", group);
    state->setAlignment(Qt::AlignCenter);
    state->setStyleSheet("font-weight: bold; color: #64748b;");
    remaining->setAlignment(Qt::AlignCenter);
    remaining->setStyleSheet("font-size: 22px; font-weight: bold; color: #64748b;");
    card_layout->addWidget(state);
    card_layout->addWidget(remaining);

    SignalCard card;
    card.state_label = state;
    card.remaining_label = remaining;
    cards_.emplace(signal.id, card);
    grid->addWidget(group, static_cast<int>(index / 3), static_cast<int>(index % 3));
  }

  layout->addLayout(grid);
  layout->addStretch(1);

  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &SpatPanel::updateWidgets);
  connect(
    this, &SpatPanel::signalUpdated, this, &SpatPanel::applySignalUpdate,
    Qt::QueuedConnection);
  update_timer_->start(100);
}

void SpatPanel::onInitialize()
{
  const auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }

  node_ = node_abstraction->get_raw_node();
  subscribe();
}

void SpatPanel::subscribe()
{
  if (!node_) {
    return;
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions();

  subscription_ = node_->create_subscription<v2i_spat_msgs::msg::SpatPacket>(
    topic_, rclcpp::QoS(10).best_effort().durability_volatile(),
    [this](const v2i_spat_msgs::msg::SpatPacket::SharedPtr message) {
      handleSpat(message);
    }, subscription_options);
}

void SpatPanel::handleSpat(const v2i_spat_msgs::msg::SpatPacket::SharedPtr message)
{
  for (const auto & intersection : message->spat.intersections) {
    for (const auto & state : intersection.states) {
      const int signal_id = static_cast<int>(intersection.intersection_id) * 10 + state.signal_group;
      const auto card_it = cards_.find(signal_id);
      if (card_it == cards_.end() || state.events.empty()) {
        continue;
      }

      const auto event_it = std::find_if(
        state.events.begin(), state.events.end(), [](const auto & event) {
          return event.event_state != v2i_spat_msgs::msg::MovementEvent::EVENT_UNKNOWN;
        });
      if (event_it == state.events.end()) {
        continue;
      }

      const double remaining = remainingSeconds(
        intersection.moy, intersection.time_stamp, *event_it);
      Q_EMIT signalUpdated(signal_id, event_it->event_state, remaining);
    }
  }
}

double SpatPanel::remainingSeconds(
  uint32_t moy, uint16_t timestamp_ms,
  const v2i_spat_msgs::msg::MovementEvent & event)
{
  Q_UNUSED(moy);
  const double current = static_cast<double>(timestamp_ms) / 100.0;

  double end = std::numeric_limits<double>::quiet_NaN();
  if (event.timing.has_min_end_time) {
    end = event.timing.min_end_time;
  } else if (event.timing.has_max_end_time) {
    end = event.timing.max_end_time;
  }
  if (std::isnan(end)) {
    return 0.0;
  }

  double delta = end - current;
  if (delta < 0.0) {
    delta += kWrapWindowDeciseconds;
  }
  return std::max(0.0, delta / 10.0);
}

void SpatPanel::applySignalUpdate(int signal_id, int event_state, double remaining_seconds)
{
  const auto card_it = cards_.find(signal_id);
  if (card_it == cards_.end()) {
    return;
  }

  auto & card = card_it->second;
  card.event_state = event_state;
  card.remaining_seconds = remaining_seconds;
  card.has_data = true;
  card.last_update.restart();
  card.state_label->setText(eventStateName(event_state));
  const auto color = eventStateColor(event_state);
  card.state_label->setStyleSheet(
    QString("color: %1; font-weight: bold;").arg(color));
  card.remaining_label->setText(QString("%1 s").arg(remaining_seconds, 0, 'f', 1));
  card.remaining_label->setStyleSheet(
    QString("font-size: 22px; font-weight: bold; color: %1;").arg(color));
}

void SpatPanel::updateWidgets()
{
  for (auto & [signal_id, card] : cards_) {
    Q_UNUSED(signal_id);
    if (!card.has_data || !card.last_update.isValid()) {
      continue;
    }
    if (card.last_update.elapsed() > kStaleTimeoutMs) {
      setCardStale(card);
      continue;
    }

    const double displayed = std::max(
      0.0, card.remaining_seconds - static_cast<double>(card.last_update.elapsed()) / 1000.0);
    card.remaining_label->setText(QString("%1 s").arg(displayed, 0, 'f', 1));
  }
}

void SpatPanel::setCardStale(SignalCard & card)
{
  card.has_data = false;
  card.state_label->setText("STALE");
  card.state_label->setStyleSheet("color: #64748b; font-weight: bold;");
  card.remaining_label->setText("--");
  card.remaining_label->setStyleSheet(
    "font-size: 22px; font-weight: bold; color: #64748b;");
}

QString SpatPanel::eventStateName(int event_state)
{
  switch (event_state) {
    case v2i_spat_msgs::msg::MovementEvent::STOP_AND_REMAIN:
      return "STOP";
    case v2i_spat_msgs::msg::MovementEvent::PROTECTED_MOVEMENT_ALLOWED:
    case v2i_spat_msgs::msg::MovementEvent::PERMISSIVE_MOVEMENT_ALLOWED:
      return "GO";
    case v2i_spat_msgs::msg::MovementEvent::PROTECTED_CLEARANCE:
      return "CLEARANCE";
    default:
      return "UNKNOWN";
  }
}

QString SpatPanel::eventStateColor(int event_state)
{
  switch (event_state) {
    case v2i_spat_msgs::msg::MovementEvent::STOP_AND_REMAIN:
      return "#ef4444";
    case v2i_spat_msgs::msg::MovementEvent::PROTECTED_MOVEMENT_ALLOWED:
    case v2i_spat_msgs::msg::MovementEvent::PERMISSIVE_MOVEMENT_ALLOWED:
      return "#22c55e";
    case v2i_spat_msgs::msg::MovementEvent::PROTECTED_CLEARANCE:
      return "#f59e0b";
    default:
      return "#94a3b8";
  }
}

void SpatPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", QString::fromStdString(topic_));
}

void SpatPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic_config;
  if (config.mapGetString("Topic", &topic_config)) {
    topic_ = topic_config.toStdString();
    if (topic_label_) {
      topic_label_->setText(QString("SPAT topic: %1").arg(QString::fromStdString(topic_)));
    }
    if (node_) {
      subscription_.reset();
      subscribe();
    }
  }
}

}  // namespace v2i_spat_rviz_panel

PLUGINLIB_EXPORT_CLASS(v2i_spat_rviz_panel::SpatPanel, rviz_common::Panel)