#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <QApplication>
#include <QtUiTools/QUiLoader>
#include <QFile>
#include <QPushButton>
#include <QLabel>
#include <QLCDNumber>
#include <QTimer>
#include <QPixmap>
#include <QImage>

#include <thread>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <iostream>

// --- ÉTATS GLOBAUX PARTAGÉS ---
std::atomic<bool> system_on{false};
std::atomic<bool> mirror_mode{false}; 
std::atomic<bool> is_rth{false};      
std::atomic<int> cam_focus{0}; // 0 = Leader, 1 = Follower

// Helper pour l'angle (Yaw) extrait du Quaternion d'odométrie
double get_yaw(const geometry_msgs::msg::Quaternion& q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class DroneStation : public rclcpp::Node {
public:
    DroneStation() : Node("drone_station_node") {
        
        // Configuration QoS : Crucial pour Gazebo
        // L'odométrie est souvent publiée en "Best Effort" par le bridge
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        pub_l = this->create_publisher<geometry_msgs::msg::Twist>("/model/drone_leader/cmd_vel", qos_reliable);
        pub_f = this->create_publisher<geometry_msgs::msg::Twist>("/model/drone_follower/cmd_vel", qos_reliable);
        
        // --- SUBSCRIBER LEADER (Passé en Best Effort pour l'odométrie) ---
        sub_odom_l = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/drone_leader/odometry", qos_best_effort, 
            [this](const nav_msgs::msg::Odometry::SharedPtr m) {
                std::lock_guard<std::mutex> l(mtx_odom);
                p_l_x = m->pose.pose.position.x; 
                p_l_y = m->pose.pose.position.y;
                alt_l = m->pose.pose.position.z; 
                yaw_l = get_yaw(m->pose.pose.orientation);
                std::cout <<"ALTITUDE RECUE :" << alt_l << std::endl;
            });
            
        // --- SUBSCRIBER FOLLOWER ---
        sub_odom_f = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/drone_follower/odometry", qos_best_effort, 
            [this](const nav_msgs::msg::Odometry::SharedPtr m) {
                std::lock_guard<std::mutex> l(mtx_odom);
                p_f_x = m->pose.pose.position.x; 
                p_f_y = m->pose.pose.position.y;
                alt_f = m->pose.pose.position.z; 
                yaw_f = get_yaw(m->pose.pose.orientation);
            });

        sub_img_l = this->create_subscription<sensor_msgs::msg::Image>("/model/drone_leader/camera", qos_sensor, 
            [this](const sensor_msgs::msg::Image::SharedPtr m) { 
                std::lock_guard<std::mutex> l(mtx_img_l); img_l = m; 
            });
        
        sub_img_f = this->create_subscription<sensor_msgs::msg::Image>("/model/drone_follower/camera", qos_sensor, 
            [this](const sensor_msgs::msg::Image::SharedPtr m) { 
                std::lock_guard<std::mutex> l(mtx_img_f); img_f = m; 
            });
            
        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, 
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                std::lock_guard<std::mutex> l(mtx_cmd);
                // Bouton Start (Xbox: Start, PS: Options) pour activer le système
                if (msg->buttons[7] == 1 && !l_st) { system_on = !system_on; l_st = true; } else if (msg->buttons[7] == 0) l_st = false;
                
                if (!system_on) return;

                if (msg->buttons[0] == 1 && !l_a) { mirror_mode = !mirror_mode; l_a = true; } else if (msg->buttons[0] == 0) l_a = false;
                if (msg->buttons[1] == 1 && !l_b) { cam_focus = (cam_focus == 0) ? 1 : 0; l_b = true; } else if (msg->buttons[1] == 0) l_b = false;
                if (msg->buttons[3] == 1 && !l_y) { is_rth = !is_rth; l_y = true; } else if (msg->buttons[3] == 0) l_y = false;

                cmd_joy.linear.x = msg->axes[1] * 3.0;
                cmd_joy.linear.y = msg->axes[0] * 3.0;
                cmd_joy.angular.z = msg->axes[3] * 2.0;
                cmd_joy.linear.z = ((1.0f - msg->axes[5])/2.0f - (1.0f - msg->axes[2])/2.0f) * 2.5;
            });
    }

    void update_and_publish() {
        if (!system_on) return;
        geometry_msgs::msg::Twist out;
        { std::lock_guard<std::mutex> l(mtx_cmd); out = cmd_joy; }

        if (is_rth) {
            std::lock_guard<std::mutex> l(mtx_odom);
            double dx = 0.0 - p_l_x; double dy = 0.0 - p_l_y;
            geometry_msgs::msg::Twist rth;
            rth.linear.x = std::clamp((dx * std::cos(yaw_l) + dy * std::sin(yaw_l)) * 0.7, -1.5, 1.5);
            rth.linear.y = std::clamp((-dx * std::sin(yaw_l) + dy * std::cos(yaw_l)) * 0.7, -1.5, 1.5);
            rth.linear.z = std::clamp((2.0 - alt_l) * 0.8, -1.0, 1.0);
            pub_l->publish(rth);
        } 
        else if (mirror_mode) { 
            pub_l->publish(out);
            double dyaw; { std::lock_guard<std::mutex> l(mtx_odom); dyaw = yaw_l - yaw_f; }
            while (dyaw > M_PI) dyaw -= 2*M_PI; while (dyaw < -M_PI) dyaw += 2*M_PI;
            geometry_msgs::msg::Twist f_cmd = out; 
            f_cmd.angular.z += dyaw * 1.5; 
            pub_f->publish(f_cmd); 
        } 
        else { 
            if (cam_focus == 0) { pub_l->publish(out); pub_f->publish(geometry_msgs::msg::Twist()); }
            else { pub_f->publish(out); pub_l->publish(geometry_msgs::msg::Twist()); }
        }
    }

    // Accesseurs avec thread-safety
    double get_alt_l() { std::lock_guard<std::mutex> l(mtx_odom); return alt_l; }
    double get_alt_f() { std::lock_guard<std::mutex> l(mtx_odom); return alt_f; }
    double get_dist() { 
        std::lock_guard<std::mutex> l(mtx_odom); 
        return std::sqrt(std::pow(p_l_x - p_f_x, 2) + std::pow(p_l_y - p_f_y, 2) + std::pow(alt_l - alt_f, 2)); 
    }
    sensor_msgs::msg::Image::SharedPtr get_img_l() { std::lock_guard<std::mutex> l(mtx_img_l); return img_l; }
    sensor_msgs::msg::Image::SharedPtr get_img_f() { std::lock_guard<std::mutex> l(mtx_img_f); return img_f; }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_l, pub_f;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_l, sub_odom_f;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_l, sub_img_f;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    geometry_msgs::msg::Twist cmd_joy;
    sensor_msgs::msg::Image::SharedPtr img_l, img_f;
    double p_l_x=0, p_l_y=0, alt_l=0, yaw_l=0, p_f_x=0, p_f_y=0, alt_f=0, yaw_f=0;
    bool l_st=false, l_a=false, l_b=false, l_y=false;
    std::mutex mtx_odom, mtx_cmd, mtx_img_l, mtx_img_f;
};

// Conversion ROS Image vers Qt QPixmap
QPixmap rosToQt(sensor_msgs::msg::Image::SharedPtr m) {
    if (!m || m->data.empty()) return QPixmap();
    try {
        QImage qim(&m->data[0], m->width, m->height, m->step, QImage::Format_RGB888);
        return QPixmap::fromImage(qim.rgbSwapped()); // rgbSwapped() si les couleurs sont inversées (BGR/RGB)
    } catch (...) { return QPixmap(); }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); 
    QApplication app(argc, argv);
    auto node = std::make_shared<DroneStation>();
    
    // Thread ROS séparé pour ne pas bloquer l'interface Qt
    std::thread ros_thread([node]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });
    ros_thread.detach();

    QUiLoader loader; 
    QFile file("/home/rabe/ros2_ws/src/drone_binome/src/drone_ui.ui");
    if (!file.open(QFile::ReadOnly)) {
        std::cerr << "ERREUR : Impossible d'ouvrir le fichier .ui ! Vérifiez le chemin." << std::endl;
        return -1;
    }
    QWidget* ui = loader.load(&file); 
    file.close();

    // --- MAPPING UI ---
    auto lbl_l = ui->findChild<QLabel*>("view_leader");
    auto lbl_f = ui->findChild<QLabel*>("view_follower");
    auto lcd_alt_l = ui->findChild<QLCDNumber*>("lcd_alt_leader");
    auto lcd_alt_f = ui->findChild<QLCDNumber*>("lcd_alt_follower");
    auto lcd_dist = ui->findChild<QLCDNumber*>("lcd_distance");
    auto btn_st = ui->findChild<QPushButton*>("btn_start");
    auto btn_l = ui->findChild<QPushButton*>("btn_leader");
    auto btn_f = ui->findChild<QPushButton*>("btn_follower");
    auto btn_rth_ui = ui->findChild<QPushButton*>("btn_rth_ui");

    // Connexions des boutons
    if(btn_l) QObject::connect(btn_l, &QPushButton::clicked, [](){ cam_focus = 0; });
    if(btn_f) QObject::connect(btn_f, &QPushButton::clicked, [](){ cam_focus = 1; });
    if(btn_rth_ui) QObject::connect(btn_rth_ui, &QPushButton::clicked, [](){ is_rth = !is_rth; });

    // Timer de mise à jour de l'UI (30ms ~ 33 FPS)
    QTimer* timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, [&]() {
        node->update_and_publish();
        
        // Mise à jour des images caméras
        auto il = node->get_img_l();
        auto iff = node->get_img_f();
        if (lbl_l && il) lbl_l->setPixmap(rosToQt(il).scaled(lbl_l->size(), Qt::KeepAspectRatio));
        if (lbl_f && iff) lbl_f->setPixmap(rosToQt(iff).scaled(lbl_f->size(), Qt::KeepAspectRatio));

        // Mise à jour des LCD (Altitude et Distance)
        if (lcd_alt_l) lcd_alt_l->display(node->get_alt_l());
        if (lcd_alt_f) lcd_alt_f->display(node->get_alt_f());
        if (lcd_dist) lcd_dist->display(node->get_dist());

        // Mise à jour visuelle du bouton Start
        if (btn_st) {
            btn_st->setText(system_on ? (mirror_mode ? "FOLLOW MODE ON" : "SYSTEM ACTIVE") : "SYSTEM PAUSED");
            btn_st->setStyleSheet(system_on ? "background-color: #2ecc71; color: white; font-weight: bold;" : "background-color: #e74c3c; color: white;");
        }
        
        // Indicateur visuel de focus caméra
        if (lbl_l) lbl_l->setStyleSheet(cam_focus == 0 ? "border: 5px solid #3498db;" : "border: 1px solid gray;");
        if (lbl_f) lbl_f->setStyleSheet(cam_focus == 1 ? "border: 5px solid #e74c3c;" : "border: 1px solid gray;");
    });

    timer->start(30);
    ui->show();
    
    int result = app.exec();
    rclcpp::shutdown();
    return result;
}