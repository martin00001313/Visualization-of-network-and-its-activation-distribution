#include "mainwindow.h"
#include "graph_viz_mgr.h"
#include <QApplication>
#include <QDir>
#include <QMessageBox>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    QMessageBox msg;
    w.show();
    try {
        QString network_name, act_node_ids, nodes_pos;
        if (4 != argc) {
            network_name = QString("network.txt");
            act_node_ids = QString("active_nodes_ids.txt");
            nodes_pos = QString("nodes_positions.txt");
        } else {
            network_name = static_cast<QString>(argv[1]);
            act_node_ids = static_cast<QString>(argv[2]);
            nodes_pos = static_cast<QString>(argv[3]);
        }
        graph_viz_mgr mgr(network_name, act_node_ids, nodes_pos,"network_activation_algo_");
        mgr.run(0.001, 0.0, 0, ACTIVATION_TYPE::ALGO_A);
    }
    catch (const nt_exception& exp) {
        msg.setText(exp.what());
        msg.exec();
        return -1;
    }
    catch (std::exception& exp) {
        msg.setText(exp.what());
        msg.exec();
        return -1;
    }
    catch (...) {
        msg.setText("Unrecognized exception");
        msg.exec();
        return -1;
    }
    msg.setText("Process passed successfully!");
    msg.exec();
    return a.exec();
}
