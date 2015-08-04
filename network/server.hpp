#ifndef SERVER_HPP
#define SERVER_HPP

#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <vector>
#include <thread>
#include <mutex>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class ServerException: public std::exception {
public:
    ServerException(std::string msg): msg(msg) { }
private:
    char const *what() const throw() {
        return msg.c_str();
    }
    std::string msg;
};

class Server {
public:
    Server(): pre_close(false),
        listen_thread(accept_thread_run, this) { }
    ~Server() {
        pre_close = true;
        listen_thread.join();
    }
    void add_on_receive_hook(std::function<void (int, void *, int)> const &func) {
        hook_mutex.lock();
        v_hooks.push_back(func);
        hook_mutex.unlock();
    }
    static void accept_thread_run(Server *server) {
        int socket_fd;
        if ((socket_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0) {
            throw "socket() failed";
        }
        sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(SERVER_PORT);
        if (bind(socket_fd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            throw "bind() failed";
        }
        if (listen(socket_fd, 10) < 0) {
            throw "listen() failed";
        }
        while (1) {
            socklen_t sin_size = sizeof(struct sockaddr_in);
            sockaddr_in client_addr;
            int accept_fd;
            if ((accept_fd = accept(socket_fd, (struct sockaddr*)&client_addr,&sin_size)) == -1 ) {
                throw "accept error!";
                continue;
            }
            std::printf("Server: new connection from %s\n",(char*)inet_ntoa(client_addr.sin_addr));
            server->client_mutex.lock();
            server->v_client.push_back(accept_fd);
            server->v_client_thread.push_back(new std::thread(receive_client_run, server, accept_fd));
            server->client_mutex.unlock();
        }
        for (size_t i = 0; i < server->v_client_thread.size(); i++) {
            server->v_client_thread[i]->join();
        }
    }
    static void receive_client_run(Server *server, int fd) {
        char ch;
        std::vector<char> data;
        while (1) {
            if (read(fd, &ch, 1) < 0) {
                throw("read() error!");
            } else {
                if (ch == '\n') {
                    server->on_client_message(fd, data.data(), data.size());
                    data.clear();
                } else {
                    data.push_back(ch);
                }
            }
        }
    }
    void send_broadcast(void *buf, int len) {
        client_mutex.lock();
        for (size_t i = 0; i < v_client.size(); i++) {
            write(v_client[i], buf, len);
        }
        client_mutex.unlock();
    }
    void on_client_message(int fd, void *buf, int len) {
        if (buf != 0 && fd >= 0)
            std::cout << "on_client_message: " << len << std::endl;
        hook_mutex.lock();
        for (size_t i = 0; i < v_hooks.size(); i++) {
            v_hooks[i](fd, buf, len);
        }
        hook_mutex.unlock();
    }
    bool pre_close;
    std::thread listen_thread;
    std::mutex client_mutex;
    std::vector<int> v_client;
    std::vector<std::thread *> v_client_thread;
private:
    enum { SERVER_PORT = 5678 };
    std::mutex hook_mutex;
    std::vector<std::function<void (int, void *, int)> > v_hooks;
};

#endif // SERVER_HPP
