//
// Copyright (c) 2016-2019 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// Official repository: https://github.com/boostorg/beast
//

//------------------------------------------------------------------------------
//
// Example: WebSocket server, asynchronous
//
//------------------------------------------------------------------------------

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include "rapidjson/document.h" // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

// structer for system data
enum SystemState
{
    POWER_OFF,
    READY,
    IN_EXECUTION,
    RECOVERY,
    ERROR
};

enum CommandType{
    NONE,
    JOG,
    HAND_CONTROL,
};

enum OperationModeState{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};


struct SystemData
{
    SystemState getSystemState() const { return system_state; }
    void setSystemState(SystemState state) { system_state = state; }
    void powerOn() { request = system_state == SystemState::POWER_OFF ? 1 : 0; }
    void powerOff() { request = system_state == SystemState::READY ? -1 : 0; }
    int request = 0;

private:
    SystemState system_state = SystemState::POWER_OFF;
};

struct AppData
{
    void setZero()
    {
        for (int jnt_ctr = 0; jnt_ctr < 4; jnt_ctr++)
        {
            actual_position[jnt_ctr] = 0;
            actual_velocity[jnt_ctr] = 0;
            actual_torque[jnt_ctr] = 0;
            cart_pos[3] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
            drive_operation_mode = OperationModeState::POSITION_MODE;
            switched_on = false;
        }
    }

    double actual_position[4];
    double actual_velocity[4];
    double actual_torque[4];
    double cart_pos[3];
    double target_position[4];
    double target_velocity[4];
    double target_torque[4];
    OperationModeState drive_operation_mode;
    bool switched_on;

};

struct CommandData
{
    void setJog(int index, int dir, int mode)
    {
        this->type = CommandType::JOG;
        jog_data.index = index - 1;
        jog_data.dir = dir;
        jog_data.type = mode;
    }
    void setHandControl()
    {
        this->type = CommandType::HAND_CONTROL;
    }
    void setNone(){
        this->type = CommandType::NONE;
    }

    CommandType type;
    struct
    {
        int index;
        int dir;
        int type;
    } jog_data;
    struct
    {
        int type;
        double goal_position[4];
    } move_to_data;
};

/* pointer to shared memory object */
// int* ptr2;
double *ptrSimRobot;

SystemData *systemDataPtr;
AppData *appDataPtr;
CommandData *commandDataPtr;

// -------------- Inconming data Parser --------------------
class ParseData
{
public:
    ParseData() {}

    void parse(const rapidjson::Document &document)
    {
        if (document.HasMember("system_data"))
        {
            const rapidjson::Value &system_data = document["system_data"];
            try
            {
                setSystemData(system_data);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        if (document.HasMember("command_data"))
        {
            const rapidjson::Value &command_data = document["command_data"];
            try
            {
                setCommandData(command_data);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
            // std::cout << "data read\n";
        }
    }

private:
    void setSystemData(const rapidjson::Value &system_data)
    {
        if (!system_data["power_on"].IsBool())
        {
            throw std::runtime_error("power_on is not a boolean");
        }
        std::cout << "Change power button : " << system_data["power_on"].GetBool() << "\n";
        if (system_data["power_on"].GetBool())
        {
            //
            systemDataPtr->powerOn();
        }
        else
        {
            systemDataPtr->powerOff();
        }
    }

    void setCommandData(const rapidjson::Value &command_data)
    {
        int command_type = command_data["type"].GetInt();
        switch (command_type)
        {
        case CommandType::JOG:
        {
            // std::cout << "jog command\n";
            int mode = command_data["mode"].GetInt();
            int index = command_data["index"].GetInt();
            int dir = command_data["direction"].GetInt();
            commandDataPtr->setJog(index, dir, mode);
            break;
        }
        case CommandType::HAND_CONTROL:
        {

            // std::cout << "Hand Control command\n";
            commandDataPtr->setHandControl();
            break;
        }
        default:
            break;
        }
    }
};

class SerializeData
{
public:
    SerializeData() : writter(sb) {}

    std::string serialize(rapidjson::Document &document)
    {
        rapidjson::Value &robot_state = document["current_state"];
        setRobotState(robot_state);

        rapidjson::Value &system_data = document["system_state"];
        setSystemState(system_data);
        sb.Clear();
        writter.Reset(sb);
        // rapidjson::PrettyWriter<rapidjson::StringBuffer> writter(sb);
        document.Accept(writter);

        return std::string(sb.GetString());
    }

private:
    rapidjson::StringBuffer sb;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writter;

    void setRobotState(rapidjson::Value &document)
    {
        rapidjson::Value &position = document["positions"];
        rapidjson::Value &velocity = document["velocities"];
        rapidjson::Value &torque = document["torque"];
        for (rapidjson::SizeType jnt_cnt = 0; jnt_cnt < 6; jnt_cnt++)
        {
            std::string name = "joint" + std::to_string(jnt_cnt + 1);
            position[name.c_str()] = appDataPtr->actual_position[jnt_cnt];
            velocity[name.c_str()] = appDataPtr->actual_velocity[jnt_cnt];
            torque[name.c_str()] = appDataPtr->actual_torque[jnt_cnt];
        }
    }

    void setSystemState(rapidjson::Value &document)
    {
        document["power_on_status"] = systemDataPtr->getSystemState();
    }
};

//------------------------------------------------------------------------------

// Report a failure
void fail(beast::error_code ec, char const *what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}

// Echoes back all received WebSocket messages
class session : public std::enable_shared_from_this<session>
{
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    ParseData data_parser_;
    SerializeData data_serializer_;
    rapidjson::Document data_incoming_;
    rapidjson::Document data_outgoing_;

public:
    // Take ownership of the socket
    explicit session(tcp::socket &&socket)
        : ws_(std::move(socket))
    {
        // Set outgoing data json

        data_outgoing_.Parse("{\"current_state\" : {\"positions\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }, \"velocities\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }, \"torque\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }}, \"system_state\": { \"power_on_status\":0 } }");

        std::cout << "{\"current_state\" : {\"positions\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }, \"velocities\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }, \"torque\" : { \"joint1\":0, \"joint2\":0, \"joint3\":0, \"joint4\":0, \"joint5\":0, \"joint6\":0 }}, \"system_state\": { \"power_on_status\":0 } }"
                  << "\n";
        // data_outgoing_.Parse("{}");
        // rapidjson::Document pos1, pos2, pos3, pos4, pos5, pos6, position, current_state, power_on_status ,system_state;
        // position.Parse("{}");
        // current_state.Parse("{}");
        // system_state.Parse("{}");
        // pos1.SetDouble(0); pos1.SetDouble(0); pos2.SetDouble(0); pos3.SetDouble(0); pos4.SetDouble(0); pos5.SetDouble(0);
        // power_on_status.SetInt(PowerOnState::POWER_OFF);

        // rapidjson::Document::AllocatorType& position_allocator = position.GetAllocator();
        // position.AddMember("joint1", pos1, position_allocator);
        // position.AddMember("joint2", pos2, position_allocator);
        // position.AddMember("joint3", pos3, position_allocator);
        // position.AddMember("joint4", pos4, position_allocator);
        // position.AddMember("joint5", pos5, position_allocator);
        // position.AddMember("joint6", pos6, position_allocator);
        // current_state.AddMember("positions",position, current_state.GetAllocator());

        // system_state.AddMember("power_on_status", power_on_status, system_state.GetAllocator());

        // data_outgoing_.AddMember("current_state",current_state, data_outgoing_.GetAllocator());
        // data_outgoing_.AddMember("system_state", system_state, data_outgoing_.GetAllocator());
    }

    // Get on the correct executor
    void
    run()
    {
        // We need to be executing within a strand to perform async operations
        // on the I/O objects in this session. Although not strictly necessary
        // for single-threaded contexts, this example code is written to be
        // thread-safe by default.
        net::dispatch(ws_.get_executor(),
                      beast::bind_front_handler(
                          &session::on_run,
                          shared_from_this()));
    }

    // Start the asynchronous operation
    void
    on_run()
    {
        std::cout << "WebSocekt stated...\n";
        // Set suggested timeout settings for the websocket
        ws_.set_option(
            websocket::stream_base::timeout::suggested(
                beast::role_type::server));

        // Set a decorator to change the Server of the handshake
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::response_type &res)
            {
                res.set(http::field::server,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-server-async");
            }));
        // Accept the websocket handshake
        ws_.async_accept(
            beast::bind_front_handler(
                &session::on_accept,
                shared_from_this()));
    }

    void
    on_accept(beast::error_code ec)
    {
        /*   // uncomment this
        if(ec)
            return fail(ec, "accept");
        */

        // Read a message
        do_read();
        do_write();
    }

    void
    do_read()
    {
        // Read a message into our buffer
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &session::on_read,
                shared_from_this()));
    }

    void
    do_write()
    {

        // Echo the message
        ws_.text(ws_.got_text());
        ws_.async_write(
            net::buffer(data_serializer_.serialize(data_outgoing_)),
            beast::bind_front_handler(
                &session::on_write,
                shared_from_this()));
    }

    void
    on_read(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        // This indicates that the session was closed
        if (ec == websocket::error::closed)
            return;

        if (ec)
            return fail(ec, "read");

        // std::cout<<"buffer_ "<<beast::buffers_to_string(buffer_.data())<<std::endl;

        std::string rec_string = beast::buffers_to_string(buffer_.data());
        // std::cout << "data :" << rec_string << "\n";
        data_incoming_.Parse<0>(rec_string.c_str()).HasParseError();

        data_parser_.parse(data_incoming_);

        // Clear the buffer
        buffer_.consume(buffer_.size());

        // Do another read
        do_read();
    }

    void
    on_write(
        beast::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if (ec)
            return fail(ec, "write");

        usleep(1000);

        // Do another read
        do_write();
    }
};

//------------------------------------------------------------------------------

// Accepts incoming connections and launches the sessions
class listener : public std::enable_shared_from_this<listener>
{
    net::io_context &ioc_;
    tcp::acceptor acceptor_;

public:
    listener(
        net::io_context &ioc,
        tcp::endpoint endpoint)
        : ioc_(ioc), acceptor_(ioc)
    {
        beast::error_code ec;

        // Open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec)
        {
            fail(ec, "open");
            return;
        }

        // Allow address reuse
        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec)
        {
            fail(ec, "set_option");
            return;
        }

        // Bind to the server address
        acceptor_.bind(endpoint, ec);
        if (ec)
        {
            fail(ec, "bind");
            return;
        }

        // Start listening for connections
        acceptor_.listen(
            net::socket_base::max_listen_connections, ec);
        if (ec)
        {
            fail(ec, "listen");
            return;
        }
    }

    // Start accepting incoming connections
    void
    run()
    {
        do_accept();
    }

private:
    void
    do_accept()
    {
        // The new connection gets its own strand
        acceptor_.async_accept(
            net::make_strand(ioc_),
            beast::bind_front_handler(
                &listener::on_accept,
                shared_from_this()));
    }

    void
    on_accept(beast::error_code ec, tcp::socket socket)
    {
        if (ec)
        {
            fail(ec, "accept");
        }
        else
        {
            // Create the session and run it
            std::make_shared<session>(std::move(socket))->run();
        }

        // Accept another connection
        do_accept();
    }
};

void createSharedMemory(int &shm_fd, const char *name, int size)
{
    shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory object.");
    }
    ftruncate(shm_fd, size);
}

void mapSharedMemory(void *&ptr, int shm_fd, int size)
{
    ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }
}

void initializeSharedData()
{
    appDataPtr->setZero();
    systemDataPtr->setSystemState(SystemState::POWER_OFF);
    systemDataPtr->request = 0;
    commandDataPtr->type = CommandType::NONE;
}

void configureSharedMemory()
{
    int shm_fd_systemData;
    int shm_fd_appData;
    int shm_fd_commandData;

    createSharedMemory(shm_fd_systemData, "SystemData", sizeof(SystemData));
    createSharedMemory(shm_fd_appData, "AppData", sizeof(AppData));
    createSharedMemory(shm_fd_commandData, "CommandData", sizeof(CommandData));

    mapSharedMemory((void *&)systemDataPtr, shm_fd_systemData, sizeof(SystemData));
    mapSharedMemory((void *&)appDataPtr, shm_fd_appData, sizeof(AppData));
    mapSharedMemory((void *&)commandDataPtr, shm_fd_commandData, sizeof(CommandData));

    initializeSharedData();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    // Check command line arguments.
    if (argc != 4)
    {
        std::cerr << "Usage: websocket-server-async <address> <port> <threads>\n"
                  << "Example:\n"
                  << "    websocket-server-async 0.0.0.0 8080 1\n";
        return EXIT_FAILURE;
    }

    configureSharedMemory();

    auto const address = net::ip::make_address(argv[1]);
    auto const port = static_cast<unsigned short>(std::atoi(argv[2]));
    auto const threads = std::max<int>(1, std::atoi(argv[3]));

    std::cout << "address : " << address << std::endl;

    // The io_context is required for all I/O
    net::io_context ioc{threads};

    // std::cout<<"line 512 : "<<std::endl;
    // Create and launch a listening port
    std::make_shared<listener>(ioc, tcp::endpoint{address, port})->run();

    // std::cout<<"line 271 : "<<std::endl;

    // Run the I/O service on the requested number of threads
    std::vector<std::thread> v;
    v.reserve(threads - 1);
    for (auto i = threads - 1; i > 0; --i)
        v.emplace_back(
            [&ioc]
            {
                ioc.run();
            });

    // std::cout<<"line 283 : "<<std::endl;
    ioc.run();

    return EXIT_SUCCESS;
}
