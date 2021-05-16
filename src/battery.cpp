#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "fit.h"

// https://batteryuniversity.com/learn/article/discharge_methods
enum DischargeModel
{
    Exponential,
    Linear,
    Inverse
};

using namespace std;

// Node parameters
string discharge_model;
float max_voltage;
float min_voltage;
float base_voltage;
int initial_percent;
int discharge_current;
int recharge_current;
int base_power_consumption;
int motors_power_consumption;
int num_batteries;
string cmd_vel_topic;
bool verbose;

// Battery variables
int total_discharge_current;
float total_power;
float voltage;
float percent;
float power;
bool recharging;
float discharge_rate;
float recharge_rate;
enum DischargeModel model;
string model_name;
Equation eq;

float map_number(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void recharge_callback(const std_msgs::Bool::ConstPtr &msg)
{
    recharging = msg->data;
    if (recharging)
        recharge_rate = recharge_current / (float)(60 * 60 * 1000);
    else
        recharge_rate = 0;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double linear = sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y + msg->linear.z * msg->linear.z);
    double angular =
        sqrt(msg->angular.x * msg->angular.x + msg->angular.y * msg->angular.y + msg->angular.z * msg->angular.z);

    if (linear > 0 || angular > 0)
        discharge_rate = (motors_power_consumption + base_power_consumption) / (float)(60 * 60 * 1000);
    else
        discharge_rate = base_power_consumption / (float)(60 * 60 * 1000);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery");
    ros::NodeHandle n("~");

    // Get the parameters for this node
    n.param<string>("discharge_model", discharge_model, "exponential");
    n.param<float>("max_voltage", max_voltage, 12.5f);
    n.param<float>("min_voltage", min_voltage, 11.5f);
    n.param<float>("base_voltage", base_voltage, 12.0f);
    n.param<int>("initial_percent", initial_percent, 100);
    n.param<int>("discharge_current", discharge_current, 7000);
    n.param<int>("recharge_current", recharge_current, 2400);
    n.param<int>("base_power_consumption", base_power_consumption, 200);
    n.param<int>("motors_power_consumption", motors_power_consumption, 5000);
    n.param<int>("num_batteries", num_batteries, 1);
    n.param<string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
    n.param<bool>("verbose", verbose, false);

    // Fix the parameter values

    if (discharge_model == "exponential")
    {
        model = Exponential;
        model_name = "Exponential";
    }
    else if (discharge_model == "linear")
    {
        model = Linear;
        model_name = "Linear";
    }
    else if (discharge_model == "inverse")
    {
        model = Inverse;
        model_name = "Inverse";
    }
    else
    {
        ROS_INFO("Invalid discharge model (%s), setting to default", discharge_model.c_str());
        model = Linear;
        model_name = "Linear";
    }

    if (max_voltage < 0)
    {
        ROS_INFO("Invalid max voltage (%f), fixing it", max_voltage);
        max_voltage = 0;
    }
    if (min_voltage < 0)
    {
        ROS_INFO("Invalid min voltage (%f), fixing it", min_voltage);
        min_voltage = 0;
    }

    if (min_voltage > max_voltage)
    {
        ROS_INFO("Inverted min/max voltages (%f, %f), swapping it", min_voltage, max_voltage);
        float tmp = min_voltage;
        min_voltage = max_voltage;
        max_voltage = tmp;
    }

    if (base_voltage < 0 || base_voltage < min_voltage)
    {
        ROS_INFO("Invalid base voltage (%f), fixing it", base_voltage);
        base_voltage = min_voltage;
    }
    else if (base_voltage > max_voltage)
    {
        ROS_INFO("Invalid base voltage (%f), fixing it", base_voltage);
        base_voltage = max_voltage;
    }

    if (initial_percent < 0)
    {
        ROS_INFO("Invalid initial percent (%d), fixing it", initial_percent);
        initial_percent = 0;
    }
    else if (initial_percent > 100)
    {
        ROS_INFO("Invalid initial percent (%d), fixing it", initial_percent);
        initial_percent = 100;
    }

    if (discharge_current < 0)
    {
        ROS_INFO("Invalid discharge current (%d), fixing it", discharge_current);
        discharge_current = 0;
    }
    if (recharge_current < 0)
    {
        ROS_INFO("Invalid recharge current (%d), fixing it", recharge_current);
        recharge_current = 0;
    }
    if (base_power_consumption < 0)
    {
        ROS_INFO("Invalid base power consumption (%d), fixing it", base_power_consumption);
        base_power_consumption = 0;
    }
    if (motors_power_consumption < 0)
    {
        ROS_INFO("Invalid motors power consumption (%d), fixing it", motors_power_consumption);
        motors_power_consumption = 0;
    }
    if (num_batteries < 1)
    {
        ROS_INFO("Invalid number os batteries (%d), fixing it", num_batteries);
        num_batteries = 1;
    }

    total_discharge_current = num_batteries * discharge_current;
    total_power = (total_discharge_current / 1000.0f) * base_voltage;

    // Fit equations
    float d_voltage = max_voltage - min_voltage;
    vector<FloatPoint> data;
    int eq_deg = 1;

    switch (model)
    {
        case Linear:
            eq_deg = 1;
            data.push_back(FloatPoint(total_power, max_voltage));
            data.push_back(FloatPoint(total_power * 0.75f, min_voltage + d_voltage * 0.75f));
            data.push_back(FloatPoint(total_power * 0.5f, min_voltage + d_voltage * 0.5f));
            data.push_back(FloatPoint(total_power * 0.25f, min_voltage + d_voltage * 0.25f));
            data.push_back(FloatPoint(0, min_voltage));
            break;
        case Exponential:
            eq_deg = 4;
            data.push_back(FloatPoint(total_power, max_voltage));
            data.push_back(FloatPoint(total_power * 0.9f, min_voltage + d_voltage * 0.98f));
            data.push_back(FloatPoint(total_power * 0.8f, min_voltage + d_voltage * 0.95f));
            data.push_back(FloatPoint(total_power * 0.7f, min_voltage + d_voltage * 0.90f));
            data.push_back(FloatPoint(total_power * 0.6f, min_voltage + d_voltage * 0.85f));
            data.push_back(FloatPoint(total_power * 0.5f, min_voltage + d_voltage * 0.80f));
            data.push_back(FloatPoint(total_power * 0.4f, min_voltage + d_voltage * 0.75f));
            data.push_back(FloatPoint(total_power * 0.3f, min_voltage + d_voltage * 0.70f));
            data.push_back(FloatPoint(total_power * 0.2f, min_voltage + d_voltage * 0.30f));
            data.push_back(FloatPoint(total_power * 0.1f, min_voltage + d_voltage * 0.01f));
            data.push_back(FloatPoint(0, min_voltage));
            break;
        case Inverse:
            eq_deg = 7;
            data.push_back(FloatPoint(total_power, max_voltage));
            data.push_back(FloatPoint(total_power * 0.9f, min_voltage + d_voltage * 0.70f));
            data.push_back(FloatPoint(total_power * 0.8f, min_voltage + d_voltage * 0.50f));
            data.push_back(FloatPoint(total_power * 0.7f, min_voltage + d_voltage * 0.30f));
            data.push_back(FloatPoint(total_power * 0.6f, min_voltage + d_voltage * 0.20f));
            data.push_back(FloatPoint(total_power * 0.5f, min_voltage + d_voltage * 0.10f));
            data.push_back(FloatPoint(total_power * 0.4f, min_voltage + d_voltage * 0.05f));
            data.push_back(FloatPoint(total_power * 0.3f, min_voltage + d_voltage * 0.03f));
            data.push_back(FloatPoint(total_power * 0.2f, min_voltage + d_voltage * 0.02f));
            data.push_back(FloatPoint(total_power * 0.1f, min_voltage + d_voltage * 0.01f));
            data.push_back(FloatPoint(0, min_voltage));
            break;
    }

    // Fit
    eq.fit(eq_deg, data);

    ROS_INFO("Battery model = %s", model_name.c_str());
    ROS_INFO("Max voltage = %fV", max_voltage);
    ROS_INFO("Min voltage = %fV", min_voltage);
    ROS_INFO("Base voltage = %fV", base_voltage);
    ROS_INFO("Initial charge percent = %d%%", initial_percent);
    ROS_INFO("Discharge current = %dmAh", discharge_current);
    ROS_INFO("Recharge current = %dmAh", recharge_current);
    ROS_INFO("Base power consumption = %dmAh", base_power_consumption);
    ROS_INFO("Motors power consumption = %dmAh", motors_power_consumption);
    ROS_INFO("Number of batteries = %d", num_batteries);
    ROS_INFO("Velocity topic = %s", cmd_vel_topic.c_str());
    ROS_INFO("Equation = %s", eq.toString().c_str());

    ROS_INFO("Total discharge current = %dmAh", total_discharge_current);
    ROS_INFO("Total power = %dWh", (int)total_power);

    // Calculates the battery discharge and recharge at every second
    ros::Rate lr(1);

    // The power publisher
    ros::Publisher power_pub = n.advertise<std_msgs::Int32>("power", 1);
    // The voltage publisher
    ros::Publisher voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    // The percent publisher
    ros::Publisher percent_pub = n.advertise<std_msgs::Float32>("percent", 1);
    // The percent publisher
    ros::Publisher recharging_pub = n.advertise<std_msgs::Bool>("recharging", 1);
    // The recharge subscriber
    ros::Subscriber recharge_sub = n.subscribe("recharge", 1, recharge_callback);
    // Subscribe the velocity topic
    ros::Subscriber cmd_vel_sub = n.subscribe(cmd_vel_topic, 1, &cmd_vel_callback);

    // Set variables
    percent = initial_percent;
    power = map_number(percent, 0, 100, 0, total_power);

    recharging = false;
    discharge_rate = base_power_consumption / (float)(60 * 60 * 1000);
    recharge_rate = 0;

    while (ros::ok())
    {
        power -= discharge_rate * base_voltage;
        power += recharge_rate * base_voltage;
        if (power > total_power)
            power = total_power;
        else if (power < 0)
            power = 0;

        voltage = eq.evaluate(power);

        if (voltage < min_voltage)
            voltage = min_voltage;
        else if (voltage > max_voltage)
            voltage = max_voltage;

        percent = map_number(voltage, min_voltage, max_voltage, 0, 100);

        if (verbose)
        {
            ROS_INFO("Discharge rate = %fmAs", (discharge_rate * 1000));
            ROS_INFO("Recharge rate = %fmAs", (recharge_rate * 1000));
            ROS_INFO("Power = %dWh", (int)power);
            ROS_INFO("Voltage = %fV", voltage);
            ROS_INFO("Percent = %d%%", percent);
        }

        std_msgs::Int32 power_msg;
        power_msg.data = (int)power;

        std_msgs::Float32 voltage_msg;
        voltage_msg.data = voltage;

        std_msgs::Float32 percent_msg;
        percent_msg.data = percent;

        std_msgs::Bool recharging_msg;
        recharging_msg.data = recharging;

        power_pub.publish(power_msg);
        voltage_pub.publish(voltage_msg);
        percent_pub.publish(percent_msg);
        recharging_pub.publish(recharging_msg);

        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}