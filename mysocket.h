#include <stdio.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <errno.h>
#include <vector>   //for using vectors instead of arrays
#include <map>
#include <array>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <condition_variable>
#include "include/crow.h"
#include <unordered_set>
#include <mutex>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <websocketpp/client.hpp>
#include <chrono>
#include <queue>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <ZeroErr_control.h>

#include <websocketpp/client.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

using namespace std;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

// pull out the type of messages sent by our config
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;


#define PORT     "45680"
#define HOST_IP  "192.168.1.144"
#define MAXLINE 1024

using namespace std;
using json = nlohmann::json;



const char *msg;
int sockfd, portno, n;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[1048576];

void initsocket();
void on_open(client* c, websocketpp::connection_hdl hdl);
void on_fail(client* c, websocketpp::connection_hdl hdl);
void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg);
void on_close(client* c, websocketpp::connection_hdl hdl);
string toRawString(string const& in);
int jjson(string diu);

// variable
bool isSettingChanged;
bool isConnected;
json* motor_setting;
string robot_cmd;

int arm_tor[5];
int arm_ang[5];



vector<int> cur_arr1;
vector<int>	cur_arr2;
vector<int>	cur_arr3;
vector<int>	cur_arr4;
vector<int>	cur_arr5;
vector<int>	cur_arr6;


// setup USBCAN


