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
    Client(char const *server_name) {
        struct hostent *host;
        sockaddr_in server_addr;
        int socket_fd;
        if ((host = gethostbyname(server_name)) == NULL) {
            throw "Gethostname error";
        }
        if ((socket_fd=socket(AF_INET,SOCK_STREAM,0)) == -1) { // AF_INET:Internet; SOCK_STREAM:TCP
            throw "Socket Error";
        }
        bzero(&server_addr,sizeof(server_addr)); // 初始化,置0
        server_addr.sin_family = AF_INET;        // IPV4
        server_addr.sin_port = htons(SERVER_PORT);  // (将本机器上的short数据转化为网络上的short数据)端口号
        server_addr.sin_addr = *((in_addr *)host->h_addr); // IP地址
        if (connect(socket_fd, (struct sockaddr *)(&server_addr), sizeof(struct sockaddr)) == -1) {
            throw "Connect Error";
        }
    }
    void add_on_receive_hook(std::function<void (int, void *, int)> const &func) {
        hook_mutex.lock();
        v_hooks.push_back(func);
        hook_mutex.unlock();
    }

private:
    enum { SERVER_PORT = 5678 };
    std::mutex hook_mutex;
    std::vector<std::function<void (int, void *, int)> > v_hooks;
};

#endif // CLIENT_HPP
