#include <gafro/gafro.hpp>
#include <gafro_control/MotorAdmittanceController.hpp>
#include <gafro_control/RobotModel.hpp>
#include <gafro_robot_descriptions/FrankaEmikaRobot.hpp>
#include <orwell/orwell.hpp>
#include <sackmesser_runtime/Interface.hpp>

int main(int argc, char **argv)
{
    auto interface = sackmesser::runtime::Interface::create(argc, argv);

    orwell::ControllerManager<7> controller_manager(interface, gafro_control::RobotModel<7>::create<gafro::FrankaEmikaRobot>());

    auto admittance_controller = controller_manager.getTorqueController("admittance_controller");

    for (unsigned i = 0; i < 100; ++i)
    {
        orwell::RobotState<7> robot_state;
        robot_state.setPosition(orwell::RobotState<7>::Vector::Random());
        robot_state.setVelocity(orwell::RobotState<7>::Vector::Random());
        robot_state.setTorque(orwell::RobotState<7>::Vector::Random());

        Eigen::Vector<double, 7> torque_command = admittance_controller->getControlCommand(robot_state);

        std::cout << torque_command.transpose() << std::endl;
    }

    return 0;
}