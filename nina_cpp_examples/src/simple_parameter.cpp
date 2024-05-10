#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>
#include <memory>

// class is created and rclcpp node is inherted and is defined as public ie) the public memebers of the node class can be accessed thorughout the SimpleParameter class

class SimpleParameter : public rclcpp:: Node
{
    //constructor is created for the SimpleParameter class (public - can be called outside the class)
public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_parameter", 14);
        declare_parameter<std::string>("simple_string_parameter","Nina");

        //when any of our parameters are changed this cb function will be called
        // add_on_set_parameters_callback - is a function of the ndoe class and it is invoked whenever the parameters inside the node are changed
        // here the std::bind is used to bind the callback function to the class instance
        // 
        param_callback_handle = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, std::placeholders::_1));
    }

// since the cb is private to the class we used private mode to access it
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        // auto is used to automatically deduce the type of parameters from the 'parameter' vector 
        for (const auto& param: parameters)
        {
            if(param.get_name() == "simple_int_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(),"Param simple_int_parameter chnaged: New value is:"<< param.as_int());
                result.successful = true;
            }

             if(param.get_name() == "simple_string_parameter" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(),"Param simple_string_parameter chnaged: New value is:"<< param.as_string());
                result.successful = true;
            }
        }
        return result;
    }
};

//argc and argv enable cmd line parameters to be passed to the program
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}