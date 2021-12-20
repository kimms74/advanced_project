#pragma once

#include <vector>
#include <functional>
#include <set>
#include <math.h>
#include <algorithm>
#include <iostream>



namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };

    struct Vec2f
    {
        double x, y;

        bool operator == (const Vec2f& coordinates_);
    };

    struct Obs2i
    {
        double x, y, d; //x,y: coordinate, s: size
    };

    struct Vec3d
    {
        double x, y, z;

        bool operator == (const Vec3d& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;
    using CoordinateListf = std::vector<Vec2f>;
    using PathList = std::vector<Vec3d>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node* parent;

        Node(Vec2i coord_, Node* parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void Simplepath(CoordinateList& path, Obs2i& obs1, Obs2i& obs2, Obs2i& obs3);
        void setWorldSize(Vec2i worldSize_, int grid_multiplier_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        void findPath(CoordinateList& path_, Vec2i source_, Vec2i target_);
        void changeGridObs(const AStar::Obs2i &robs1_, const AStar::Obs2i& robs2_, const AStar::Obs2i& robs3_, AStar::Obs2i& gobs1_, AStar::Obs2i& gobs2_, AStar::Obs2i& gobs3_,const double &safety_value_);
        void addCollision(AStar::Obs2i obs1_, AStar::Obs2i obs2_, AStar::Obs2i obs3_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        void printMap();
        void changeRealPath(const CoordinateList& path_, PathList& real_path_, double scale, Vec3d start_, Vec3d target_);
        double PointLineDistance(double x1, double y1, double x2, double y2, double rx, double ry);
        double distance(double x1, double y1, double x2, double y2);



    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        std::vector<std::vector<std::string>> grid_map;
        Vec2i worldSize;
        uint directions;
        int grid_multiplier;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}