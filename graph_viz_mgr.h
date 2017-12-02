#ifndef GRAPH_VIZ_MGR_H
#define GRAPH_VIZ_MGR_H

#include <QString>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <memory>
#include <functional>

#include "exception.h"

template <class T>
struct object_pair_cmp
{
   bool operator() (const std::pair<T, T>& p1, const std::pair<T, T> & p2) const noexcept
   {
       auto& lf_max = std::max(p1.first, p1.second);
       auto& lf_min = std::min(p1.first, p1.second);
       auto& rh_max = std::max(p2.first, p2.second);
       auto& rh_min = std::min(p2.first, p2.second);
       return lf_min == rh_min ? lf_max < rh_max : lf_min < rh_min;
   }
};

enum ACTIVATION_TYPE
{
  NONE = 0x0,
  ALGO_A = 0x1,
  ALGO_B = 0x2,
  ALGO_C = 0x4,
  ALGO_D = 0x8,
  ALGO_E = 0x10,
  ALGO_F = 0x20,
  ALL = ALGO_A | ALGO_B | ALGO_C | ALGO_D | ALGO_E | ALGO_F
};

class graph_viz_mgr
{
public:
    graph_viz_mgr(const QString& mtx, const QString& actives, const QString& positions, const QString& output_file);

    void run(double mu, double lambda, unsigned max_step, int algo_type = ACTIVATION_TYPE::ALL);

private:
    void base_init();
    void init_activation_status();
    void init_node_positions();
    void init_scene();
    void reset();
    void runA();
    void runB();
    void runC();
    void runD();
    void runE();
    void runF();

private:
    size_t m_act_nodes_count;
    std::set<unsigned> m_nodes;
    std::unique_ptr<QGraphicsScene> m_scene;
    std::unordered_map<unsigned, std::unordered_set<unsigned>> m_node_to_neighbors;
    std::vector<std::vector<bool>> m_mtx;
    std::unordered_map<unsigned, std::pair<double, double>> m_node_to_pos;
    std::vector<bool> m_activation_state;
    std::map<std::pair<unsigned, unsigned>, QGraphicsLineItem*, object_pair_cmp<unsigned>> m_lines;
    std::map<unsigned, QGraphicsEllipseItem*> m_elipses;
    std::vector<std::pair<uint, uint>> m_last_activated_edges;
    double m_mu;
    double m_lambda;
    unsigned m_max_step;
    const QString m_mtx_path, m_act_file_path, m_pos_file_path, m_output_file;

private:
    std::vector<unsigned> get_active_neighbors(unsigned idx);
};

#endif // GRAPH_VIZ_MGR_H
