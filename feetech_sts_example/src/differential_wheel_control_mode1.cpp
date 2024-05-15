#include <feetech_sts_example/differential_wheel_control.hpp>

#define max(a, b) ((a < b) ? b : a)
#define min(a, b) ((a < b) ? a : b)
#define clip(a, max_val, min_val) (max(min(a, max_val), min_val))

WheelSpeedCmd convert_to_speed(geometry_msgs::msg::Twist cmd_vel)
{
  WheelSpeedCmd ret = {};
  ret.right = STEP_RATIO * (linear_coef * cmd_vel.linear.x + angular_coef_r * cmd_vel.angular.z);
  ret.left = -STEP_RATIO * (linear_coef * cmd_vel.linear.x + angular_coef_l * cmd_vel.angular.z);
  ret.right = clip(ret.right, MAX_SPEED, -MAX_SPEED);
  ret.left = clip(ret.left, MAX_SPEED, -MAX_SPEED);
  return ret;
}

void move_cmd(const u_char id_right, const u_char id_left,
              std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler,
              const geometry_msgs::msg::Twist &cmd_vel)
{
  auto speed = convert_to_speed(cmd_vel);
  u_char id_list[2] = {(u_char)id_right, (u_char)id_left};
  int16_t vel_list[2] = {speed.right, speed.left};
  u_short acc_list[2] = {(u_short)ACC, (u_short)ACC};
  // int16_t goal_list[2] = {1000, -1000};
  // packet_handler->syncWritePosEx(id_list, sizeof(id_list), goal_list, vel_list, acc_list);
  packet_handler->syncWriteSpd(id_list, sizeof(id_list), vel_list, acc_list);

}

void stop(
  const u_char id_right, const u_char id_left,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  u_char id_list[2] = {(u_char)id_right, (u_char)id_left};
  int16_t vel_list[2] = {0, 0};
  u_short acc_list[2] = {0, 0};
  int16_t goal_list[2] = {0, 0};
  packet_handler->syncWritePosEx(id_list, sizeof(id_list), goal_list, vel_list, acc_list);
}

void set_wheel_mode(
  const u_char id_right, const u_char id_left,
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  packet_handler->unlockEprom(id_right);
  packet_handler->setWheelMode(id_right);
  // packet_handler->setOpenLoopWheelMode(id_right);
  packet_handler->setTorque(id_right, true);
  packet_handler->calbrationOffset(id_right);
  packet_handler->lockEprom(id_right);

  packet_handler->unlockEprom(id_left);
  packet_handler->setWheelMode(id_left);
  // packet_handler->setOpenLoopWheelMode(id_left);
  packet_handler->setTorque(id_left, true);
  packet_handler->calbrationOffset(id_left);
  packet_handler->lockEprom(id_left);
}

Status get_status(const u_char id_right, const u_char id_left, std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler)
{
  Status status;
  {
    int16_t val0 = 0;
    int16_t val1 = 0;
    if (packet_handler->readSpd(id_right, val0) && packet_handler->readSpd(id_left, val1))
    {
      status.speed.is_read = true;
      status.speed.right = feetech_sts_interface::STS3032::data2angle(val0);
      status.speed.left = feetech_sts_interface::STS3032::data2angle(val1);
    }
    if (packet_handler->readPos(id_right, val0) && packet_handler->readPos(id_left, val1))
    {
      status.pos.is_read = true;
      status.pos.right = feetech_sts_interface::STS3032::data2angle(val0);
      status.pos.left = feetech_sts_interface::STS3032::data2angle(val1);
    }
  }
  return status;
}


void print_status(const Status status)
{
  if (status.speed.is_read)
  {
    std::cout << "speed right/ left: " << status.speed.right << " / " << status.speed.left << " [deg/sec]" << std::endl;
  }
  else
  {
    std::cout << "failed to read speed." << std::endl;
  }
  if (status.pos.is_read)
  {
    std::cout << "pos right/ left: " << status.pos.right << " / " << status.pos.left << std::endl;
  }
  else
  {
    std::cout << "failed to read pos." << std::endl;
  }
  std::cout << std::endl;
}

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cout << "Usage: " << argv[0] << " <id right> <id left> <port_name (/dev/ttyUSB0)> <baudrate (1000000)>" << std::endl;
    return EXIT_FAILURE;
  }
  const int RIGHT_ID = std::atoi(argv[1]);
  const int LEFT_ID = std::atoi(argv[2]);
  std::string port_name = argv[3];
  const int baudrate = std::stoi(argv[4]);
  std::cout << "Closed Loop Mode" << std::endl;
  std::cout << "RIGHT_ID: " << RIGHT_ID << std::endl;
  std::cout << "LEFT_ID: " << LEFT_ID << std::endl;
  std::cout << "port_name: " << port_name << std::endl;
  std::cout << "baudrate: " << baudrate << std::endl;

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open())
  {
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "set closed loop wheel mode." << std::endl;
  set_wheel_mode(RIGHT_ID, LEFT_ID, packet_handler);
  std::cout << "done." << std::endl;

  // Command stop
  stop(RIGHT_ID, LEFT_ID, packet_handler);
  // To stabilize subsequent readings
  {
    for (size_t i = 0; i < 5; ++i)
    {
      get_status(RIGHT_ID, LEFT_ID, packet_handler);
    }
  }
  // wait 1sec
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  geometry_msgs::msg::Twist cmd_vel;

  std::cout << "MOVE FORWARD." << std::endl;
  cmd_vel.linear.x = 5.0;  // [cm/sec]
  cmd_vel.angular.z = 0.0; // [rad/sec]
  move_cmd(RIGHT_ID, LEFT_ID, packet_handler, cmd_vel);

  for (size_t i = 0; i < 10; ++i)
  {
    auto status = get_status(RIGHT_ID, LEFT_ID, packet_handler);
    print_status(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  std::cout << "STOP." << std::endl;
  stop(RIGHT_ID, LEFT_ID, packet_handler);

  size_t counter = 0;
  auto prev_status = get_status(RIGHT_ID, LEFT_ID, packet_handler);
  while (counter < 10)
  {
    auto status = get_status(RIGHT_ID, LEFT_ID, packet_handler);
    print_status(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto delta0 = prev_status.pos.right - status.pos.right;
    const auto delta1 = prev_status.pos.left - status.pos.left;
    if (abs(delta0) < 0.1 && abs(delta1) < 0.1)
    {
      ++counter;
    }
    else
    {
      counter = 0;
    }
    prev_status = status;
    stop(RIGHT_ID, LEFT_ID, packet_handler);
  }

  port_handler->close();
  std::cout << "shutdown." << std::endl;
  return EXIT_SUCCESS;
}
