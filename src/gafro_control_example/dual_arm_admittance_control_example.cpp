#include <gafro/gafro.hpp>
#include <gafro_control/DualArmAbsoluteAdmittanceController.hpp>
#include <gafro_robot_descriptions/FrankaEmikaRobot.hpp>
#include <orwell/orwell.hpp>
#include <sackmesser_runtime/Interface.hpp>

int main(int argc, char **argv)
{
    auto interface = sackmesser::runtime::Interface::create(argc, argv);

    Eigen::Vector3d p1({ 0.0, 0.0, 0.0 });
    Eigen::Vector3d p2({ 1.0, 0.0, 0.0 });
    Eigen::Quaterniond o1 = Eigen::Quaterniond::UnitRandom();
    Eigen::Quaterniond o2 = Eigen::Quaterniond::UnitRandom();

    auto dual_panda = std::make_shared<gafro::DualManipulator<double, 14>>(gafro::FrankaEmikaRobot<double>(), gafro::Motor<double>(p1, o1),
                                                                           gafro::Motor<double>(p2, o2));

    gafro_control::DualArmAbsoluteAdmittanceController<7> admittance_controller(interface, "dual_arm_controller", dual_panda);

    gafro::Motor<double> absolute_target = dual_panda->getAbsoluteMotor(Eigen::Vector<double, 7>::Random(), Eigen::Vector<double, 7>::Random());

    for (unsigned i = 0; i < 100; ++i)
    {
        orwell::RobotState<14> robot_state;
        robot_state.setPosition(orwell::RobotState<14>::Vector::Random());
        robot_state.setVelocity(orwell::RobotState<14>::Vector::Random());
        robot_state.setTorque(orwell::RobotState<14>::Vector::Random());

        Eigen::Vector<double, 14> torque_command = admittance_controller.computeCommand(robot_state);

        std::cout << torque_command.transpose() << std::endl;
    }

    return 0;
}