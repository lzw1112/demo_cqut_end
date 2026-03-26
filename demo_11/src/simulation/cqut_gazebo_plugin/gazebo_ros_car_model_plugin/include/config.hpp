#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

struct Param 
{
    struct Inertia 
    {
        double m;
        double m_driver;
        double g;
        double I_z;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Inertia:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "m: %f, m_driver: %f, g: %f, I_z: %f", m, m_driver, g, I_z);
        }
    };

    struct Kinematic 
    {
        double l;
        double b_F;
        double b_R;
        double w_front;
        double l_F;
        double l_R;
        double h_cg;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Kinematic:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "l: %f, b_F: %f, b_R: %f, w_front: %f, l_F: %f, l_R: %f, h_cg: %f", l, b_F, b_R, w_front, l_F, l_R, h_cg);
        }
    };

    struct Tire 
    {
        double tire_coefficient;
        double B;
        double C;
        double D;
        double E;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Tire:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "B: %f, C: %f, D: %f, E: %f", B, C, D, E);
        }
    };

    struct Aero 
    {
        double c_down;
        double c_drag;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Aero:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "c_down: %f, c_drag: %f", c_down, c_drag);
        }
    };

    struct DriveTrain 
    {
        int    nm_wheels;
        double inertia;
        double r_dyn;
        double m_lon_add;
        double cm1;
        double cr0;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "DriveTrain:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "nm_wheels: %i, inertia: %f, r_dyn: %f, Cm1: %f, Cr0: %f", nm_wheels, inertia, r_dyn, cm1, cr0);
        }
    };

    struct TorqueVectoring 
    {
        double K_FFW;
        double K_p;
        double shrinkage;
        double K_stability;
        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "TorqueVectoring:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "K_FFW: %f, K_p: %f, shrinkage: %f, K_stability: %f", K_FFW, K_p, shrinkage, K_stability);
        }
    };

    struct Sensors 
    {
        double noise_vx_sigma;
        double noise_vy_sigma;
        double noise_r_sigma;

        void print() 
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "Sensors:");
            RCLCPP_DEBUG(rclcpp::get_logger("Param"), "noise_vx_sigma: %f, noise_vy_sigma: %f, noise_r_sigma: %f", noise_vx_sigma, noise_vy_sigma, noise_r_sigma);
        }
    };

    Inertia         inertia;
    Kinematic       kinematic;
    Tire            tire;
    Aero            aero;
    DriveTrain      driveTrain;
    TorqueVectoring torqueVectoring;
    Sensors         sensors;
};

namespace YAML 
{
template<>
struct convert<Param::Inertia> 
{
    static bool decode(const Node &node, Param::Inertia &cType) 
    {
        cType.m        = node["m"].as<double>();
        cType.m_driver = node["m_driver"].as<double>();
        cType.g        = node["g"].as<double>();
        cType.I_z      = node["I_z"].as<double>();

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED Inertia");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Kinematic> 
{
    static bool decode(const Node &node, Param::Kinematic &cType) 
    {
        cType.l       = node["l"].as<double>();
        cType.b_F     = node["b_F"].as<double>();
        cType.b_R     = node["b_R"].as<double>();
        cType.w_front = node["w_front"].as<double>();
        cType.h_cg    = node["h_cg"].as<double>();
        cType.l_F     = cType.l * (1 - cType.w_front);
        cType.l_R     = cType.l * cType.w_front;

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED Kinematic");
        cType.print();
        return true;
    }
};

inline double mean(const double a, const double b) 
{
    return (a + b) / 2.0;
}

template<>
struct convert<Param::Tire> 
{
    static bool decode(const Node &node, Param::Tire &cType) 
    {
        cType.tire_coefficient = node["tire_coefficient"].as<double>();
        cType.B = node["B"].as<double>() / cType.tire_coefficient;
        cType.C = node["C"].as<double>();
        cType.D = node["D"].as<double>() * cType.tire_coefficient;
        cType.E = node["E"].as<double>();

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED Tire");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Aero> 
{
    static bool decode(const Node &node, Param::Aero &cType) 
    {
        cType.c_down = node["C_Down"]["a"].as<double>() * node["C_Down"]["b"].as<double>() * node["C_Down"]["c"].as<double>();
        cType.c_drag = node["C_drag"]["a"].as<double>() * node["C_drag"]["b"].as<double>() * node["C_drag"]["c"].as<double>();

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED Aero");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::DriveTrain> 
{
    static bool decode(const Node &node, Param::DriveTrain &cType) 
    {
        cType.inertia   = node["inertia"].as<double>();
        cType.r_dyn     = node["r_dyn"].as<double>();
        cType.nm_wheels = node["nm_wheels"].as<int>();
        cType.cr0       = node["Cr0"].as<double>();
        cType.cm1       = node["Cm1"].as<double>();
        cType.m_lon_add = cType.nm_wheels * cType.inertia / (cType.r_dyn * cType.r_dyn);

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED DriveTrain");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::TorqueVectoring> 
{
    static bool decode(const Node &node, Param::TorqueVectoring &cType) 
    {
        cType.K_FFW       = node["K_FFW"].as<double>();
        cType.K_p         = node["K_p"].as<double>();
        cType.shrinkage   = node["shrinkage"].as<double>();
        cType.K_stability = node["K_stability"].as<double>();

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED TorqueVectoring");
        cType.print();
        return true;
    }
};

template<>
struct convert<Param::Sensors> 
{
    static bool decode(const Node &node, Param::Sensors &cType) 
    {
        cType.noise_vx_sigma = node["noise_vx_sigma"].as<double>();
        cType.noise_vy_sigma = node["noise_vy_sigma"].as<double>();
        cType.noise_r_sigma  = node["noise_r_sigma"].as<double>();

        RCLCPP_DEBUG(rclcpp::get_logger("Param"), "LOADED Sensors");
        cType.print();
        return true;
    }
};

}

inline void initParam(Param &param, std::string &yaml_file) 
{
    YAML::Node config = YAML::LoadFile(yaml_file);

    RCLCPP_DEBUG(rclcpp::get_logger("Param"), "STARTING THIS YAML CraP: %s ********************************", yaml_file.c_str());
    const auto car    = config["car"];
    param.inertia         = car["inertia"].as<Param::Inertia>();
    param.kinematic       = car["kinematics"].as<Param::Kinematic>();
    param.tire            = car["tire"].as<Param::Tire>();
    param.aero            = car["aero"].as<Param::Aero>();
    param.driveTrain      = car["drivetrain"].as<Param::DriveTrain>();
    param.torqueVectoring = car["torque_vectoring"].as<Param::TorqueVectoring>();
}

inline void initParamSensors(Param &param, std::string &yaml_file) 
{
    YAML::Node config = YAML::LoadFile(yaml_file);

    RCLCPP_DEBUG(rclcpp::get_logger("Param"), "STARTING SENSORS THIS YAML: %s ********************************", yaml_file.c_str());
    const auto car    = config["sensors"];
    param.sensors = car["velocity"].as<Param::Sensors>();
}
