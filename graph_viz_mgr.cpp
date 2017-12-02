#include "graph_viz_mgr.h"

#include <fstream>
#include <random>
#include <QFile>
#include <QPainter>
#include <QTextStream>
#include <cassert>
#include <math.h>

namespace {
  constexpr double R = 24.0;

  template <typename T>
  typename T::const_iterator get_random_iterator(const T& set)
  {
      if (set.empty()) {
          return set.cend();
      }
      static std::random_device rd;
      static std::mt19937_64 gen(rd());
      std::uniform_int_distribution<std::size_t> dist(0, set.size() - 1);
      int P = dist(gen);
      int cur_val = 0;
      auto it = set.cbegin();
      for (it = set.cbegin(); cur_val != P; ++cur_val, ++it) {}

      return it;
  }

  template <>
  std::vector<unsigned>::const_iterator get_random_iterator(const std::vector<unsigned>& set)
  {
      if (set.empty()) {
          return set.cend();
      }
      static std::random_device rd;
      static std::mt19937 gen(rd());
      std::uniform_int_distribution<> dist(0, set.size()-1);
      return set.begin() + dist(gen);
  }

  void save_image(const std::unique_ptr<QGraphicsScene>& scene, const QString& file_path, unsigned idx)
  {
      scene->clearSelection();
      scene->setSceneRect(scene->itemsBoundingRect().marginsAdded(QMarginsF(R, R, R, R)));

      QImage image(scene->sceneRect().size().toSize(), QImage::Format_ARGB32);
      image.fill(Qt::white);

      QPainter painter(&image);
      scene->render(&painter);
      image.save(file_path + QString::number(idx).rightJustified(5, '0') + ".png");
  }

} //unnamed namespace


graph_viz_mgr::graph_viz_mgr(const QString& mtx, const QString& actives, const QString& positions, const QString& output_file)
    : m_act_nodes_count(0), m_mtx_path(mtx)
    , m_act_file_path(actives), m_pos_file_path(positions), m_output_file(output_file)
{
    base_init();
    init_activation_status();
    init_node_positions();
    init_scene();
}

std::vector<unsigned>  graph_viz_mgr::get_active_neighbors(unsigned idx)
{
    if(idx >= m_nodes.size()) {
        return std::vector<unsigned>();
    }
    std::vector<unsigned> act_nhbr;
    const auto& nhbr = m_node_to_neighbors[idx];
    for (auto nhg : nhbr) {
        if (!m_activation_state[nhg]) {
            act_nhbr.push_back(nhg);
        }
    }
    return act_nhbr;
}

void graph_viz_mgr::base_init()
{
    QFile file(m_mtx_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw nt_exception(std::string("File not exists!"));
        return;
    }

    QTextStream	s(&file);
    while (!s.atEnd()) {
        QString line = s.readLine();
        if (line.isEmpty()) {
            return;
        }
        auto node_pair = line.split(" ");
        if (2 != node_pair.count()) {
            throw nt_exception(std::string("Incorrect file format."));
        }
        unsigned node1 = node_pair.at(0).toUInt();
        unsigned node2 = node_pair.at(1).toUInt();
        m_nodes.insert(node1);
        m_nodes.insert(node2);
        m_node_to_neighbors[node1].insert(node2);
        m_node_to_neighbors[node2].insert(node1);
    }
    file.close();

    const auto nodes_count = m_nodes.size();
    m_mtx.resize(nodes_count, std::vector<bool>(nodes_count, false));
    for (auto& s : m_node_to_neighbors) {
        const auto base_node = s.first;
        for (auto cur_node : s.second) {
            m_mtx[base_node][cur_node] = true;
            m_mtx[cur_node][base_node] = true;
        }
    }
}

void graph_viz_mgr::init_node_positions()
{
    QFile pos_file(m_pos_file_path);
    if (!pos_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw nt_exception(std::string("File not exists!"));
    }

    QTextStream	s(&pos_file);
    while (!s.atEnd()) {
        QString line = s.readLine();
        if (line.isEmpty()) {
            continue;
        }
        QStringList tokens = line.split(" ");
        if (3 != tokens.size()) {
            continue;
        }
        auto id = tokens.at(0).toUInt();
        auto point1 = tokens.at(1).toDouble();
        auto point2 = tokens.at(2).toDouble();
        if (!m_node_to_pos.emplace(id , std::make_pair(point1, point2)).second) {
            throw nt_exception(std::string("Incorrect file format!"));
        }
    }
    pos_file.close();
}

void graph_viz_mgr::init_activation_status()
{
    m_activation_state.resize(m_nodes.size(), false);
    QFile activation_file(m_act_file_path);
    if (!activation_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw nt_exception(std::string("File not exists!"));
    }

    QTextStream s(&activation_file);
    std::unordered_set<int> unique_id;
    while (!s.atEnd()) {
        QString line = s.readLine();
        QStringList tokens = line.split(" ");
        if (tokens.size() != 1) {
            throw nt_exception(std::string("Incorrect file format!"));
        }
        uint node_id = tokens.at(0).toUInt();
        if (!unique_id.emplace(node_id).second) {
            throw nt_exception(std::string("Incorrect file format!"));
        }
        m_activation_state.at(node_id) = true;
        ++m_act_nodes_count;
    }
    activation_file.close();
}

void graph_viz_mgr::init_scene()
{
    m_scene.reset(new QGraphicsScene());

    QBrush def_brush(Qt::darkGray);
    QBrush act_brush(Qt::red);

    QPen inact_pen(Qt::darkGray);
    QPen act_pen(Qt::red);

    QPen red_line(0xFFB6C1);  //, 1.314159265);
    QPen black_line(Qt::black);
    const auto nodes_count = m_nodes.size();
    for (unsigned i = 0; i < nodes_count; ++i) {
        auto pair1 = m_node_to_pos[i];
        if (m_activation_state[i]) {
            auto* ptr = m_scene->addEllipse(QRectF(pair1.first, pair1.second, R, R), act_pen, act_brush);
            m_elipses.emplace(i, ptr);
        } else {
            auto* ptr = m_scene->addEllipse(QRectF(pair1.first, pair1.second, R, R), inact_pen, def_brush);
            m_elipses.emplace(i, ptr);
        }
        QGraphicsTextItem* text = m_scene->addText(QString::number(i));
        text->setPos(pair1.first, pair1.second);
        text->setDefaultTextColor(QColor(0, 0, 0, 255));
        auto& neighbors = m_node_to_neighbors[i];
        for (auto j : neighbors) {
            if (m_mtx[i][j]) {
                auto pair2 = m_node_to_pos[j];
                auto x = m_lines.emplace(std::make_pair(i,j), nullptr);
                if (x.second) {
                    if (m_activation_state[i] && m_activation_state[j]){
                        x.first->second = m_scene->addLine(pair1.first + R/2, pair1.second + R/2, pair2.first + R/2, pair2.second + R / 2, red_line);
                    } else {
                        x.first->second = m_scene->addLine(pair1.first + R/2, pair1.second + R/2, pair2.first + R/2, pair2.second + R / 2, black_line);
                    }
                }
            }
        }
    }
}

void graph_viz_mgr::reset()
{
    m_nodes.clear();
    m_node_to_neighbors.clear();
    m_node_to_pos.clear();
    m_mtx.clear();
    m_activation_state.clear();
    m_last_activated_edges.clear();
    m_lines.clear();
    m_elipses.clear();

    base_init();
    init_activation_status();
    init_node_positions();
    init_scene();
}

void graph_viz_mgr::run(double mu, double lambda, unsigned max_step, int act_type)
{
    m_mu = mu;
    m_lambda = lambda;
    m_max_step = max_step;
    if (act_type & ACTIVATION_TYPE::ALGO_A) {
        runA();
    }
    if (act_type & ACTIVATION_TYPE::ALGO_B) {
        reset();
        runB();
    }
    if (act_type & ACTIVATION_TYPE::ALGO_C) {
        reset();
        runC();
    }
    if (act_type & ACTIVATION_TYPE::ALGO_D)
    {
        reset();
        runD();
    }
    if (act_type & ACTIVATION_TYPE::ALGO_E)
    {
        reset();
        runE();
    }
    if (act_type & ACTIVATION_TYPE::ALGO_F) {
        reset();
        runF();
    }
}

void graph_viz_mgr::runA()
{
    const QString tmp_image_file = m_output_file + "A";
    QPen red_line(QBrush(Qt::red),1.314159265);
    QPen black_line(Qt::black);
    QPen green_line(QBrush(Qt::green), 3.14159265);

    std::vector<long double> trace;
    const double P1 = m_mu / (m_mu + m_lambda);

    save_image(m_scene, tmp_image_file, 0);  //save temporary state
    uint step_count = 0;

    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::bernoulli_distribution dist(P1);

    while (0 != m_act_nodes_count && (m_max_step == 0 || step_count <= m_max_step)) {
        ++step_count;
        trace.emplace_back(static_cast<double>(m_act_nodes_count) / m_nodes.size());
        for (auto pair : m_last_activated_edges) {    // set last_activated_edge color as it was(red/black if nodes avtive/inactive appropriately)
            auto it = m_lines.find(pair);
            if (it != m_lines.end()) {
                if (m_activation_state[pair.first] && m_activation_state[pair.second]) {
                    it->second->setPen(red_line);
                } else {
                    it->second->setPen(black_line);
                }
            } else {
                assert(false);
            }
        }
        m_last_activated_edges.clear();  //should be empty before current process will started

        auto it = get_random_iterator(m_nodes);
        unsigned int cur_node = *it;
        if (!m_activation_state[cur_node]) {
               save_image(m_scene, tmp_image_file, step_count);
               continue;
        }

        if (!dist(gen)) {
            auto& neighbors = m_node_to_neighbors[cur_node];
            const auto rand_neighbor = get_random_iterator(neighbors);
            if (rand_neighbor != neighbors.cend()) {
                if (!m_activation_state[*rand_neighbor]) { //if node isn't active yet
                    m_last_activated_edges.push_back(std::make_pair(cur_node, *rand_neighbor)); //collect connect. egdes
                    m_activation_state[*rand_neighbor] = true;
                    ++m_act_nodes_count;
                    // set rand_neighbor  active color
                    auto node_iter = m_elipses.find(*rand_neighbor);
                    if (node_iter != m_elipses.end()) {
                        node_iter->second->setPen(QPen(Qt::red));
                        node_iter->second->setBrush(QBrush(Qt::red));
                    } else {
                        throw nt_exception("Container doesn't contain necessary info.");
                    }
                    unsigned rand_node = *rand_neighbor;
                    std::unordered_set<unsigned>& neighbors1 = m_node_to_neighbors[rand_node];
                    for (auto neighbor : neighbors1) {
                        if (m_activation_state[neighbor]) {
                            //if in and out nodes are active then set appropriate edge color red
                            auto it = m_lines.find(std::make_pair(rand_node, neighbor));
                            if (it != m_lines.end())
                            {
                                it->second->setPen(red_line);
                            } else {
                                throw nt_exception("Container doesn't contain necessary info.");
                            }
                        }
                    }
                }
            }
        }
        if(dist(gen)) {
            m_activation_state[cur_node] = false;
            --m_act_nodes_count;
            //set cur_node color yellow (as it already inactive) and if it has active neighbors set color of appropriate edges black
            auto it = m_elipses.find(cur_node);
            if (it != m_elipses.cend()) {
                it->second->setPen(QPen(Qt::darkGray));
                it->second->setBrush(QBrush(Qt::darkGray));
            } else {
                throw nt_exception("Container doesn't contain necessary info.");
            }
            auto& neighbors = m_node_to_neighbors[cur_node];
            for (auto ng : neighbors) {
                if (m_activation_state[ng]) { //also we can set without any check, because default color for edges is black
                    auto it = m_lines.find(std::make_pair(cur_node, ng));
                    if (it != m_lines.end()) {
                        it->second->setPen(black_line);
                    } else {
                        throw nt_exception("Container doesn't contain necessary info.");
                    }
                }
            }
        }

        for (auto& pair : m_last_activated_edges) {
            auto it = m_lines.find(pair);
            if (it != m_lines.end()) {
                it->second->setPen(green_line);
            }
            else {
                throw nt_exception("Container doesn't contain necessary info.");
            }
        }
        save_image(m_scene, tmp_image_file, step_count); //generate  unique name for new image
    }

    trace.emplace_back(static_cast<double>(m_act_nodes_count) / m_nodes.size());
    if (m_act_nodes_count != 0 && step_count < m_max_step) {
        throw nt_exception("Smth. inccorrect in implementation.");
    }
    std::ofstream fs, fs1;
    fs.open(std::string("trace_n_" + std::to_string(m_nodes.size()) + "_mu_" + std::to_string(m_mu) + "_lambda_" + std::to_string(m_lambda) + ".txt").c_str());
    fs1.open(std::string("lambda_trace_n_" + std::to_string(m_nodes.size()) + "_mu_" + std::to_string(m_mu) + "_lambda_" + std::to_string(m_lambda) + ".txt").c_str());
    for (unsigned int i = 0; i < trace.size(); ++i) {
        fs <<i <<"   "<<trace[i]<<'\n';
        fs1<<i<<"  "<<std::log10(trace[i])<<'\n';
    }
    fs.close();
    fs1.close();
    system("ffmpeg -framerate 24 -t 180 -i algoA%05d.png algoA_video.mp4");
}

void graph_viz_mgr::runB()
{
    throw nt_exception(std::string("AlgoB has not implemented yed."));
}

void graph_viz_mgr::runC()
{
    throw nt_exception(std::string("AlgoC has not implemented yed."));
}

void graph_viz_mgr::runD()
{
    throw nt_exception(std::string("AlgoD has not implemented yed."));
}

void graph_viz_mgr::runE()
{
    throw nt_exception(std::string("AlgoE has not implemented yed."));
}

void graph_viz_mgr::runF()
{
    throw nt_exception(std::string("AlgoF has not implemented yed."));
}
