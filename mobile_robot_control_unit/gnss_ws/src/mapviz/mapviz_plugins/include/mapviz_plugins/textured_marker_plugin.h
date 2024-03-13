// *****************************************************************************
//
// Copyright (c) 2014-2020, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MAPVIZ_PLUGINS__TEXTURED_MARKER_PLUGIN_H_
#define MAPVIZ_PLUGINS__TEXTURED_MARKER_PLUGIN_H_

#include <mapviz/mapviz_plugin.h>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>
#include <QColor>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <marti_visualization_msgs/msg/textured_marker.hpp>
#include <marti_visualization_msgs/msg/textured_marker_array.hpp>

#include <mapviz/map_canvas.h>

// C++ standard libraries
#include <list>
#include <map>
#include <string>
#include <vector>

// QT autogenerated files
#include "ui_textured_marker_config.h"

Q_DECLARE_METATYPE(marti_visualization_msgs::msg::TexturedMarker)

namespace mapviz_plugins
{
class TexturedMarkerPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

public:
  TexturedMarkerPlugin();
  ~TexturedMarkerPlugin() override = default;

  bool Initialize(QGLWidget * canvas) override;
  void Shutdown() override {}

  void Draw(double x, double y, double scale) override;

  void Transform() override;

  void LoadConfig(const YAML::Node & node, const std::string & path) override;
  void SaveConfig(YAML::Emitter & emitter, const std::string & path) override;

  QWidget * GetConfigWidget(QWidget * parent) override;

Q_SIGNALS:
  void MarkerReceived(marti_visualization_msgs::msg::TexturedMarker marker);

protected:
  void PrintError(const std::string & message) override;
  void PrintInfo(const std::string & message) override;
  void PrintWarning(const std::string & message) override;

protected Q_SLOTS:
  void SetAlphaLevel(int alpha);
  void SelectTopic();
  void TopicEdited();
  void ClearHistory() override;
  void ProcessMarker(marti_visualization_msgs::msg::TexturedMarker marker);

private:
  float alphaVal_;

  struct MarkerData
  {
    rclcpp::Time stamp;
    rclcpp::Time expire_time;

    float alpha_;

    std::vector<uint8_t> texture_;
    int32_t texture_id_;
    int32_t texture_size_;
    float texture_x_;
    float texture_y_;

    std::string encoding_;

    std::vector<tf2::Vector3> quad_;
    std::vector<tf2::Vector3> transformed_quad_;

    std::string source_frame_;

    bool transformed;
  };

  Ui::textured_marker_config ui_{};
  QWidget * config_widget_;

  std::string topic_;

  rclcpp::Subscription<marti_visualization_msgs::msg::TexturedMarker>::SharedPtr marker_sub_;
  rclcpp::Subscription<marti_visualization_msgs::msg::TexturedMarkerArray>::SharedPtr
    marker_arr_sub_;

  bool has_message_;

  std::map<std::string, std::map<int, MarkerData>> markers_;

  void MarkerCallback(marti_visualization_msgs::msg::TexturedMarker::ConstSharedPtr marker);

  void MarkerArrayCallback(
    marti_visualization_msgs::msg::TexturedMarkerArray::ConstSharedPtr markers);
};
}   // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__TEXTURED_MARKER_PLUGIN_H_
