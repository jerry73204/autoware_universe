// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.hpp"

#include <stdexcept>
#include <utility>

namespace autoware::cuda_pointcloud_preprocessor
{
CudaVoxelGridDownsampleFilterNode::CudaVoxelGridDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_voxel_grid_downsample_filter", node_options)
{
  // set initial parameters
  float voxel_size_x = declare_parameter<float>("voxel_size_x");
  float voxel_size_y = declare_parameter<float>("voxel_size_y");
  float voxel_size_z = declare_parameter<float>("voxel_size_z");
  int64_t max_mem_pool_size_in_byte = declare_parameter<int64_t>(
    "max_mem_pool_size_in_byte",
    1e9);  // 1GB in default
  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  cuda_voxel_grid_downsample_filter_ = std::make_unique<CudaVoxelGridDownsampleFilter>(
    voxel_size_x, voxel_size_y, voxel_size_z, max_mem_pool_size_in_byte);

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback, this, std::placeholders::_1),
      cuda_voxel_grid_downsample_filter_->stream());

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");
}

void CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // The filter states its own requirements on the input layout and throws naming the
  // first one the cloud does not meet (see
  // CudaVoxelGridDownsampleFilter::resolveInputFields). Report that and drop the
  // cloud rather than letting it unwind out of the subscription callback, and
  // throttle it: a misconfigured upstream node produces one of these per message.
  try {
    auto output_pointcloud_ptr = cuda_voxel_grid_downsample_filter_->filter(msg);
    pub_->publish(std::move(output_pointcloud_ptr));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Dropping input pointcloud: %s", e.what());
  }
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaVoxelGridDownsampleFilterNode)
