// switch (prevetter_response.code) {
//     case MediationLayerCode::Success:
//       // Invoke the visualizer to see the proposed trajectory, which will be
//       // displayed in violet. See student_game_engine_visualizer.h for other
//       // visualization options: you can visualize a short path, a single point,
//       // etc. It will be helpful to get such visual feedback on candidate
//       // trajectories. Note that there is a built-in visualizer called
//       // "ViewManager" implemented elsewhere in the game-engine code, but you
//       // don't have full control over what it displays like you do with the
//       // Student_game_engine_visualizer invoked below.
//       visualizer.drawTrajectory(trajectory);
//       quad_to_trajectory_map[quad_name] = trajectory;
//       dt_factor = dt_factor - 0.06;
//       // std::cout << "speed up! :" << dt_factor << std::endl;
//       return quad_to_trajectory_map;
//       break;
//     case MediationLayerCode::NotEnoughTrajectoryPoints: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::StartPointFarFromCurrentPosition: {
//       dt_factor = 2;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       break;
//     }
//     case MediationLayerCode::PointExceedsMapBounds: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::PointWithinObstacle: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       // std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       // std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxVelocity: {
//       std::cout << "too fast! :" << dt_factor << std::endl;
//       dt_factor = dt_factor + 0.1;
//       // std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxVelocity: {
//       std::cout << "too fast! :" << dt_factor << std::endl;
//       dt_factor = dt_factor + 0.1;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::ExceedsMaxAcceleration: {
//       std::cout << "too fast! :" << dt_factor << std::endl;
//       dt_factor = dt_factor + 0.1;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::MeanValueExceedsMaxAcceleration: {
//       std::cout << "too fast! :" << dt_factor << std::endl;
//       dt_factor = dt_factor + 0.1;
//       //   std::cout << "slow down! :" << seg_part[seg] << std::endl;
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::TimestampsNotIncreasing: {
//       std::cout << "Prevet code: " << static_cast<int>(prevetter_response.code)
//                 << std::endl;
//       std::cout << "Prevet value: " << prevetter_response.value << std::endl;
//       std::cout << "Prevet index: " << prevetter_response.index << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }
//     case MediationLayerCode::TimeBetweenPointsExceedsMaxTime: {
//       // Suppose your AP intends to submit a trajectory with a time that exceeds
//       // the maximum allowed time between points. The prevetter would catch this
//       // before you submit to the mediation layer.
//       std::cout << "Prevet: Shorten time between trajectory points."
//                 << std::endl;
//       std::cout << "Prevet: Time violation: " << prevetter_response.value
//                 << std::endl;
//       return empty_quad_to_trajectory_map;
//       break;
//     }