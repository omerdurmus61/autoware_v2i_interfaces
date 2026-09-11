#ifndef V2I_SPAT_RVIZ_PANEL__SPAT_PANEL_HPP_
#define V2I_SPAT_RVIZ_PANEL__SPAT_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include <QElapsedTimer>
#include <QLabel>
#include <QTimer>
#include <QWidget>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>

#include <v2i_spat_msgs/msg/spat_packet.hpp>

namespace v2i_spat_rviz_panel
{

class SpatPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SpatPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void updateWidgets();
  void applySignalUpdate(int signal_id, int event_state, double remaining_seconds);

Q_SIGNALS:
  void signalUpdated(int signal_id, int event_state, double remaining_seconds);

private:
  struct SignalCard
  {
    QLabel * state_label{nullptr};
    QLabel * remaining_label{nullptr};
    bool has_data{false};
    int event_state{0};
    double remaining_seconds{0.0};
    QElapsedTimer last_update;
  };

  void createUi();
  void subscribe();
  void handleSpat(const v2i_spat_msgs::msg::SpatPacket::SharedPtr message);
  void setCardStale(SignalCard & card);
  static double remainingSeconds(
    uint32_t moy, uint16_t timestamp_ms,
    const v2i_spat_msgs::msg::MovementEvent & event);
  static QString eventStateName(int event_state);
  static QString eventStateColor(int event_state);

  std::string topic_{"/v2i/spat/raw"};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<v2i_spat_msgs::msg::SpatPacket>::SharedPtr subscription_;
  QTimer * update_timer_{nullptr};
  QLabel * topic_label_{nullptr};
  std::map<int, SignalCard> cards_;
};

}  // namespace v2i_spat_rviz_panel

#endif  // V2I_SPAT_RVIZ_PANEL__SPAT_PANEL_HPP_