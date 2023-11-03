/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASplanner_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASplanner_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <fstream>
#include <iomanip>
#include <string>
#define STRING(num) #num

using namespace std;
namespace  ASplanner
{
    struct Vec2i
    {
        double x, y;
        bool operator == (const Vec2i& coordinates_);
    };
    struct Gdge_property//读取的路径信息
    {
        bool state=true;
        unsigned int collsion_id=0;
        double leng = 0;
        unsigned int source_index = 0;
        unsigned int target_index = 0;      
    };
    struct ID
    {
        unsigned int main_id;
        unsigned int sec_id;
        unsigned int last_id;
    };
    struct Car_config
    {
        unsigned int type=0;
        int index=0;
        //unsigned int target_index;
        vector<pair<int, int>> middle_point;
        double car_v;
    };
    struct G_Node//读取的节点信息
    {
        //节点名称
        ID name = {0,0,0};
        unsigned int index = 0;//节点编号
        Vec2i coordinates = { 0,0 };
        //std::vector<G_Node> link_Nodes={};
        std::vector<Gdge_property> link_edges = {};//该节点的连接情况
        
    };
    struct path_point//路径与点的连接信息
    {
        int index = 0;//索引
        G_Node GN;
        Gdge_property path;
        double start_time = 0;
        double end_time = 0;
        double spend_time = 0;
    };
    
    struct Node    //使用A*算法中节点的各种需求，包括G、H、父节点、节点信息、
    {
        double G, H;
        Vec2i coordinates;
        Node* parent;//父节点
        G_Node GN;
        Node(G_Node GN_, Node* parent_ = nullptr);//实例化产生空间
        double getScore();
    };                 
    using uint = unsigned int;
    using HeuristicFunction = std::function<double(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;
    using G_NodeList = std::vector<G_Node>;
    using pathList = std::vector<path_point>;
    using car_path = std::pair<Car_config, pathList>;
    using NodeSet = std::vector<Node*>;//存放节点地址的集合

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, uint index);
        void releaseNodes(NodeSet& nodes_);

        public:

            Generator();
            void setWorldSize(Vec2i worldSize_);
            void setDiagonalMovement(bool enable_);
            void setHeuristic(HeuristicFunction heuristic_);
            pathList findPath(G_Node source_, G_Node target_, vector<G_Node> *GNs);
            bool opposing_conflict(car_path* path,uint k ,car_path* pro_path,uint n, vector<G_Node>* GNs);
            void init_time_windows(double time_cnt,pair<Car_config, pathList> *path,vector<G_Node>*  GNs);
            bool collsion_collection(car_path* path,uint k,car_path* pro_path,uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j);
            double conflict_check(vector<pair<Car_config, pathList>>* GNLs, vector<G_Node>* GNs);
            bool colliding_conflict(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs,vector<pair<Car_config, pathList>>* paths, uint i, uint j);
            void node_conflict(car_path* path,uint k, car_path* pro_path,uint n, vector<G_Node>* GNs);
            pathList station_is_vechel(uint k, uint pro_size, path_point* point_pro, pair<Car_config, pathList>* path, vector<G_Node>* GNs);
            bool mini_distance_between_vechels(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j);
            bool node_check(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs);
            void replanning_path(car_path* path, path_point* GN_point, vector<G_Node>* GNs);
            void store_is_vechel(car_path* path, uint k, car_path* pro_path, uint n, vector<G_Node>* GNs, vector<pair<Car_config, pathList>>* paths, uint i, uint j);
            //int time_window_dan(vector<pathList>* paths, int count, vector<G_Node>* GNs, vector<Gdge_property>* GEs);
            //int A_star_time_window(vector<pathList>* paths, G_Node source_, G_Node target_, vector<G_Node>* GNs, vector<Gdge_property>* GEs);
            void addCollision(Vec2i coordinates_);
            void removeCollision(Vec2i coordinates_);
            void clearCollisions();

        private:
            HeuristicFunction heuristic;
            CoordinateList direction, walls;
            Vec2i worldSize;
            uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static double manhattan(Vec2i source_, Vec2i target_);
        static double euclidean(Vec2i source_, Vec2i target_);
        static double octagonal(Vec2i source_, Vec2i target_);
    };


}

#endif // __ ASplanner_HPP_8F637DB91972F6C878D41D63F7E7214F__
#pragma once
