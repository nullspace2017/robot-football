#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <netdb.h>
#include <vector>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

class ClientException: public std::exception {
public:
    ClientException(std::string msg): msg(msg) { }
private:
    char const *what() const throw() {
        return msg.c_str();
    }
    std::string msg;
};

class Client {
public:
    Client(char const *server_ip) {
        pre_close = false;
        connet_to_server(server_ip);
    }
    ~Client() {
        pre_close = true;
        read_server_thread->join();
    }
    void add_on_receive_hook(std::function<void (int, void const *, int)> const &func) {
        hook_mutex.lock();
        v_hooks.push_back(func);
        hook_mutex.unlock();
    }
    void send_to_server(void const *buf, int len) {
        if (write(socket_fd, buf, len) != len)
            throw ClientException("write error");
        if (write(socket_fd, "\n", 1) != 1)
            throw ClientException("write error");
    }
private:
    void connet_to_server(char const *server_ip) {
        sockaddr_in server_addr;
        if ((socket_fd = socket(AF_INET,SOCK_STREAM,0)) < 0 ) {
            throw ClientException("create socket error");
        }
        int flag = 1;
        setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, (char const *)&flag, sizeof(flag));
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
            throw ClientException("inet_pton error");
        }
        while (connect(socket_fd, (sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            std::cout << "connect failed, try again" << std::endl;
            usleep(200000);
        }
        read_server_thread = new std::thread(receive_server_run, this, socket_fd);
    }
    static void receive_server_run(Client *client, int fd) {
        char ch;
        std::vector<char> data;
        while (1) {
            if (read(fd, &ch, 1) < 0) {
                throw ClientException("read error");
            } else {
                if (ch == '\n') {
                    data.push_back('\0');
                    client->on_server_message(fd, data.data(), data.size() - 1);
                    data.clear();
                } else {
                    data.push_back(ch);
                }
            }
        }
    }
    void on_server_message(int fd, void const *buf, int len) {
        hook_mutex.lock();
        for (size_t i = 0; i < v_hooks.size(); i++) {
            v_hooks[i](fd, buf, len);
        }
        hook_mutex.unlock();
    }
private:
    enum { SERVER_PORT = 5678 };
    bool pre_close;
    int socket_fd;
    std::thread *read_server_thread;
    std::mutex hook_mutex;
    std::vector<std::function<void (int, void const*, int)> > v_hooks;
};

#endif // CLIENT_HPP
