#include "mysocket.h"
#include <websocketpp/config/asio_no_tls_client.hpp>

#include <websocketpp/client.hpp>


joint_ctrl	motor0 = joint_ctrl("L1", 1);
joint_ctrl	motor1 = joint_ctrl("L1", 2);
joint_ctrl	motor2 = joint_ctrl("L1", 3);
joint_ctrl	motor3 = joint_ctrl("L1", 4);
joint_ctrl	motor4 = joint_ctrl("L1", 5);
joint_ctrl	motor5 = joint_ctrl("L1", 6);	



std::ofstream out("example.csv");

void initsocket()
{   
    
    int m_connect = 0;			// can bus is closed
    int m_devtype = USBCAN1;
    init_USBCAN(&m_connect, m_devtype);
    //std::lock_guard<std::mutex> lk(mtx);

    motor0.motor_enable(1);
	motor0.set_angle_mode(524288, 524288, 524288);
    motor1.motor_enable(1);
	motor1.set_angle_mode(524288, 524288, 524288);
    motor2.motor_enable(1);
	motor2.set_angle_mode(524288, 524288, 524288);
    motor3.motor_enable(1);
	motor3.set_angle_mode(524288, 524288, 524288);
    motor4.motor_enable(1);
	motor4.set_angle_mode(524288, 524288, 524288);
    motor5.motor_enable(1);
	motor5.set_angle_mode(524288, 524288, 524288);

    motor0.set_speed(5242);
    motor1.set_speed(5242);
    motor2.set_speed(5242);
    motor3.set_speed(5242);
    motor4.set_speed(5242);
   
    motor0.set_target_pos(0);
    motor1.set_target_pos(0);
    motor2.set_target_pos(0);
    motor3.set_target_pos(0);
    motor4.set_target_pos(0);

    motor0.set_speed(5242);
    motor1.set_speed(10242);
    motor2.set_speed(5242);
    motor3.set_speed(10242);
    motor4.set_speed(10242);

    sleep(10);

    client c;
    
    std::string uri = "ws://192.168.1.144:45680/ws";
    
    
    try {
        // set logging policy if needed
        c.clear_access_channels(websocketpp::log::alevel::frame_header);
        c.clear_access_channels(websocketpp::log::alevel::frame_payload);
        //c.set_error_channels(websocketpp::log::elevel::none);
        
        // Initialize ASIO
        c.init_asio();
        
        // Register our handlers
        c.set_open_handler(bind(&on_open,&c,::_1));
        c.set_fail_handler(bind(&on_fail,&c,::_1));
        c.set_message_handler(bind(&on_message,&c,::_1,::_2));
        c.set_close_handler(bind(&on_close,&c,::_1));
        
        // Create a connection to the given URI and queue it for connection once
        // the event loop starts
        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        c.connect(con);
        
        // Start the ASIO io_service run loop
        c.run();
    } catch (const std::exception & e) {
        std::cout << e.what() << std::endl;
    } catch (websocketpp::lib::error_code e) {
        std::cout << e.message() << std::endl;
    } catch (...) {
        std::cout << "other exception" << std::endl;
    }
}

// Handlers
void on_open(client* c, websocketpp::connection_hdl hdl) {
    std::string msg = "Hello";
    c->send(hdl,msg,websocketpp::frame::opcode::text);
    c->get_alog().write(websocketpp::log::alevel::app, "Sent Message: "+msg);
}

void on_fail(client* c, websocketpp::connection_hdl hdl) {
    c->get_alog().write(websocketpp::log::alevel::app, "Connection Failed");
}

void print(std::vector<int> const &input)
{   

    std::cout << "diu" << endl;
    for (int i = 0; i < input.size(); i++) {
        std::cout << input.at(i) << ' ';
    }
}
 

void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg) {

    string n = msg->get_payload();
   

    std::string diumsg = "";
    c->send(hdl,diumsg,websocketpp::frame::opcode::text);
    //c->get_alog().write(websocketpp::log::alevel::app, "Sent Message: "+diumsg);

     

    //c->get_alog().write(websocketpp::log::alevel::app, "Received Reply: "+n); 
    //cout << "dllm msg: "  <<n<<"dllm length: "  <<n.length() <<endl; 

    jjson(n);
    /*
    motor0.get_recent_current();
    motor1.get_recent_current();
    motor2.get_recent_current();
    motor3.get_recent_current();
    motor4.get_recent_current();
    motor5.get_recent_current();
       
    cur_arr1.push_back(motor0.cur_current);
    cur_arr2.push_back(motor1.cur_current);
    cur_arr3.push_back(motor2.cur_current);
    cur_arr4.push_back(motor3.cur_current);
    cur_arr5.push_back(motor4.cur_current);
    cur_arr6.push_back(motor5.cur_current);
    
    print(cur_arr1);
    print(cur_arr2);
    print(cur_arr3);
    print(cur_arr4);
    print(cur_arr5);
    print(cur_arr6);
    */
}


   


void on_close(client* c, websocketpp::connection_hdl hdl) {
    c->get_alog().write(websocketpp::log::alevel::app, "Connection Closed");
    out.close();
}


string toRawString(string const& in)
{
   std::string ret = in;
   auto p = ret.find('\t');
   if ( p != ret.npos )
   {
      ret.replace(p, 1, "\\t");
   }

   return ret;
}


int jjson(string diu)
{   
    if ( diu.length() == 0){
        return 0;
    }
    if (diu.length() < 1000){ 


        // parse and serialize JSON
        json j= json::parse(diu);
        //std::cout << j["ExoRight"]["R1"]["angle"] << j["ExoRight"]["R2"]["angle"]<< "\n\n";

        string aa0=j["ExoRight"]["R1"]["angle"];
        string aa1=j["ExoRight"]["R2"]["angle"];
        string aa2=j["ExoRight"]["R3"]["angle"];
        string aa3=j["ExoRight"]["R4"]["angle"];
        string aa4=j["ExoRight"]["R5"]["angle"];
        //string aa5=j["ExoRight"]["R6"]["angle"];

        string at0=j["ExoRight"]["R1"]["torque"];
        string at1=j["ExoRight"]["R2"]["torque"];
        string at2=j["ExoRight"]["R3"]["torque"];
        string at3=j["ExoRight"]["R4"]["torque"];
        string at4=j["ExoRight"]["R5"]["torque"];
        //string at5=j["ExoRight"]["R6"]["torque"];

        
        motor0.set_target_pos(stoi(aa0)* 524288 /360);
		motor1.set_target_pos(stoi(aa2)*-524288 /360);
		motor2.set_target_pos(stoi(aa1)*524288 /360);
		motor3.set_target_pos(stoi(aa3)*-524288 /360);

		motor4.set_target_pos(stoi(aa4)*524288 /360);
		//motor5.set_target_pos(num5);
        
        
        
        
        arm_ang[0] = stoi(aa0);
        arm_ang[1] = stoi(aa1);
        arm_ang[2] = stoi(aa2);
        arm_ang[3] = stoi(aa3);
        arm_ang[4] = stoi(aa4);
        //arm_ang[5] = stoi(aa5);

        /*
        arm_tor[0] = stoi(at0);
        arm_tor[1] = stoi(at1);
        arm_tor[2] = stoi(at2);
        arm_tor[3] = stoi(at3);
        // arm_tor[4] = stoi(at4);
        //arm_tor[5] = stoi(at5);
        */
   
        
        for (const auto& e : arm_ang) {
        std::cout << e << std::endl;
        }


        for (const auto& e : arm_tor) {
        std::cout << e << std::endl;
        }
        

        return 0;
    
    }


    else return 0;

}


int main(int argc, char* argv[]) {
	
    initsocket();
    
    return 0;
}

