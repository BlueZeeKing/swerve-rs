use robotrs::FailableDefault;
use swerve_rs::Robot;

fn main() {
    robotrs::scheduler::RobotScheduler::start_robot(|| Robot::failable_default());
}
