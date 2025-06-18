#include <memory>
#include <rclcpp/parameter_value.hpp>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc,char **argv)
{
    setvbuf(stdout,NULL,_IONBF,BUFSIZ);
    rclcpp::init(argc,argv);
    auto node=rclcpp::Node::make_shared("set_and_get_parameters");

    //パラメータの宣言
    node->declare_parameter("foo",rclcpp::PARAMETER_INTEGER);
    node->declare_parameter("bar",rclcpp::PARAMETER_STRING);
    node->declare_parameter("baz",rclcpp::PARAMETER_DOUBLE);

    auto parameters_client=std::make_shared<
    rclcpp::SyncParametersClient>(node);
    while(!parameters_client->wait_for_service(1s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(node->get_logger(),"Interrupted");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(),"Waiting");
    }
    //パラメータの設定
    auto set_parameters_results=parameters_client->set_parameters({
        rclcpp::Parameter("foo",2),
        rclcpp::Parameter("bar","hello"),
        rclcpp::Parameter("baz",3.14)
    });
    for(auto & result:set_parameters_results){
        if(!result.successful){
            RCLCPP_ERROR(node->get_logger(),"Failed: %s",
        result.reason.c_str());
        }
    }

    std::stringstream ss;
    //パラメータの取得
    for(auto &parameter:parameters_client->get_parameters(
        {"foo","bar","baz"})){
            //パラメータ名とパラメータの型名のロギング
            ss<<"\nPrameter name:"<<parameter.get_name();
            ss<<"\nPrameter value ("<<parameter.get_type_name()
            <<"):"<<parameter.value_to_string();
        }
        RCLCPP_INFO(node->get_logger(),"%s",ss.str().c_str());
    rclcpp::shutdown();
    return 0;
}