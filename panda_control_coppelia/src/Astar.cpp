#include "../include/Astar.h"

using namespace std::placeholders;

double AStar::Generator::PointLineDistance(double x1, double y1, double x2, double y2, double rx, double ry) {
    double a = (y2 - y1);
    double b = -(x2 - x1);
    double c = ((x2 - x1) * y2 - (y2 - y1) * x2) ;

    return abs(a * rx + b * ry + c) / (sqrt(pow(a, 2) + pow(b, 2)));
}

double AStar::Generator::distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

void AStar::Generator::Simplepath(CoordinateList& path, Obs2i& obs1, Obs2i& obs2, Obs2i& obs3 ) {
    //std::cout << "before erase : " << path.size() << std::endl;
    //double r1, r2, r3;
    //r1 = obs1.d / 2;
    //r2 = obs2.d / 2;
    //r3 = obs3.d / 2;

    double a, b, c;
    double r_x, r_y, r_0;
    double x_i, y_i,  x_f, y_f;
    std::vector<double> d_col(3);
    std::vector<Obs2i> obss;
    obss.push_back(obs1);
    obss.push_back(obs2);
    obss.push_back(obs3);
    int target_x, target_y;
    target_x = path.back().x;
    target_y = path.back().y;

    for (int i = 0; i < path.size(); i++) {
        if (i == 0) {
            grid_map[path[i].x][path[i].y] = "@";
        }
        if (i == path.size() - 1) {
            continue;
        }

        x_i = path[i].x;
        y_i = path[i].y;
        x_f = path[i + 1].x;
        y_f = path[i + 1].y;
        //std::cout << path[i].x << " " << path[i].y << std::endl;
        for (int j = 0; j < obss.size(); j++)
        {
            r_x = obss[j].x;
            r_y = obss[j].y;
            r_0 = (obss[j].d / 2.0);

            // collision check 
            //std::cout << x_i << " " << y_i << " " << x_f << " " << y_f << " " << r_x << " " << r_y << std::endl;
            d_col[j] = PointLineDistance(x_i, y_i, x_f, y_f, r_x, r_y);
            //std::cout << "d_col" << j << ": " << d_col[j] << ", r_0 : " << r_0 << std::endl;
            
            if (d_col[j] < r_0) {
                if ((sqrt(pow(distance(x_i, y_i, r_x, r_y), 2) - pow(d_col[j], 2)) > distance(x_i, y_i, x_f, y_f) || (sqrt(pow(distance(x_f, y_f, r_x, r_y), 2) - pow(d_col[j], 2)) > distance(x_i, y_i, x_f, y_f)))) {
                    //std::cout << "fakeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" << std::endl;

                    if (distance(x_f, y_f, r_x, r_y) > distance(x_i, y_i, r_x, r_y)) {
                        d_col[j] = distance(x_i, y_i, r_x, r_y);
                    }
                    else {
                        d_col[j] = distance(x_f, y_f, r_x, r_y);
                    }
                }
            }
        }
        //change radius 2 to another value
        double radius_gain = 2;
            if (d_col[0] > (obss[0].d / radius_gain) && d_col[1] > (obss[1].d / radius_gain) && d_col[2] > (obss[2].d / radius_gain)) {
                //std::cout << "iter : " << i << std::endl;
                grid_map[path[i + 1].x][path[i + 1].y] = "^";
                path.erase(path.begin() + i + 1);
                i--;
            }
    }
    grid_map[target_x][target_y] = "@";

    //r_x = obs1.x;
    //r_y = obs1.y;
    //r_0 = r1;
    //for (int i = 0; i < path.size(); i++) {
    //    if (i == path.size() - 1) {
    //        continue;
    //    }
    //    x_i = path[i].x;
    //    y_i = path[i].y;
    //    x_f = path[i + 1].x;
    //    y_f = path[i + 1].y;

    //    // nearest obstacle select
    //    if (distance(x_f, y_f, r_x, r_y) > distance(x_f, y_f, obs2.x, obs2.y)) {
    //        r_x = obs2.x;
    //        r_y = obs2.y;
    //        r_0 = r2;
    //    }
    //    else if (distance(x_f, y_f, r_x, r_y) > distance(x_f, y_f, obs3.x, obs3.y)) {
    //        r_x = obs3.x;
    //        r_y = obs3.y;
    //        r_0 = r3;
    //    }
    //    double a, b, c, cos_c;
    //    a = distance(x_i, y_i, x_f, y_f);
    //    b = distance(x_f, y_f, r_x, r_y);
    //    c = distance(r_x, r_y, x_i, y_i);
    //    cos_c = (pow(c, 2) - pow(a, 2) - pow(b, 2)) / (-2 * a * b);

    //    std::cout << "r_0 : " << r_0 << "\t PLD : " << PointLineDistance(x_i, y_i, x_f, y_f, r_x, r_y) << std::endl;

    //    if (cos_c <= 0) {
    //        std::cout << "iter : " << i << std::endl;
    //        grid_map[path[i + 1].x][path[i + 1].y] = "0";
    //        path.erase(path.begin() + i + 1);
    //        i--;
    //    }
    //    else if (r_0 < PointLineDistance(x_i, y_i, x_f, y_f, r_x, r_y)) {
    //        std::cout << "iter : " << i << std::endl;
    //        grid_map[path[i + 1].x][path[i + 1].y] = "0";
    //        path.erase(path.begin() + i + 1);
    //        i--;
    //    }
    //}

    //std::cout << "after erase : " << path.size() << std::endl;
}

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Vec2f::operator == (const Vec2f& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Vec3d::operator == (const Vec3d& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        //{ 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        //{ -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 },
        //{ -1, -2 }, { 1, 2 }, { -1, 2 }, { 1, -2 }
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
        
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_, int grid_multiplier_)
{
    worldSize.x = worldSize_.x * grid_multiplier_;
    worldSize.y = worldSize_.y * grid_multiplier_;
    grid_multiplier = grid_multiplier_;

    grid_map.resize(worldSize.x, std::vector<std::string>(worldSize.y, "^"));
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    //directions = (enable_ ? 12 : 4);
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::changeGridObs(const AStar::Obs2i& robs1_, const AStar::Obs2i& robs2_, const AStar::Obs2i& robs3_, AStar::Obs2i& gobs1_, AStar::Obs2i& gobs2_, AStar::Obs2i& gobs3_,const double &safety_value_)
{
    AStar::Vec2f map_center{ 0.58,0 };
    gobs1_.x = (0.68 - (robs1_.x + map_center.x)) * 100 * grid_multiplier;
    gobs1_.y = (0.2 - (robs1_.y + map_center.y)) * 100 * grid_multiplier;
    gobs2_.x = (0.68 - (robs2_.x + map_center.x)) * 100 * grid_multiplier;
    gobs2_.y = (0.2 - (robs2_.y + map_center.y)) * 100 * grid_multiplier;
    gobs3_.x = (0.68 - (robs3_.x + map_center.x)) * 100 * grid_multiplier;
    gobs3_.y = (0.2 - (robs3_.y + map_center.y)) * 100 * grid_multiplier;

    gobs1_.d = (robs1_.d + safety_value_) * 100 * grid_multiplier;
    gobs2_.d = (robs2_.d + safety_value_) * 100 * grid_multiplier;
    gobs3_.d = (robs3_.d + safety_value_) * 100 * grid_multiplier;

}


void AStar::Generator::addCollision(AStar::Obs2i obs1_, AStar::Obs2i obs2_, AStar::Obs2i obs3_)
{
    for (int i = 0; i < worldSize.x; ++i)
    {
        for (int j = 0; j < worldSize.y; ++j)
        {
            //int distance_1 = sqrt(pow(i / double(grid_multiplier) - obs1_.x, 2) + pow(j / double(grid_multiplier) - obs1_.y, 2));
            //int distance_2 = sqrt(pow(i / double(grid_multiplier) - obs2_.x, 2) + pow(j / double(grid_multiplier) - obs2_.y, 2));
            //int distance_3 = sqrt(pow(i / double(grid_multiplier) - obs3_.x, 2) + pow(j / double(grid_multiplier) - obs3_.y, 2));
            double distance_1 = sqrt(pow(i - obs1_.x, 2) + pow(j - obs1_.y, 2));
            double distance_2 = sqrt(pow(i - obs2_.x, 2) + pow(j - obs2_.y, 2));
            double distance_3 = sqrt(pow(i - obs3_.x, 2) + pow(j - obs3_.y, 2));
            if (distance_1 <= (obs1_.d / 2) || distance_2 <= (obs2_.d / 2) || distance_3 <= (obs3_.d / 2))
            {
                walls.push_back({ i,j });
                grid_map[i][j] = "1";
            }
        }
    }
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

void AStar::Generator::printMap()
{
    for (int i = 0; i < worldSize.x; ++i)
    {
        for (int j = 0; j < worldSize.y; ++j)
        {
            std::cout << grid_map[i][j] << " ";
            if (j == (worldSize.y - 1))
            {
                std::cout << std::endl;
            }
        }
    }
}

void AStar::Generator::findPath(CoordinateList& path_, Vec2i source_, Vec2i target_)
{
    //Vec2i start{ source_.x * grid_multiplier - 1, source_.y * grid_multiplier - 1 }; //change!!
    //Vec2i finish{ target_.x * grid_multiplier - 1, target_.y * grid_multiplier - 1 };

    Vec2i start{ source_.x, source_.y}; //change!!
    Vec2i finish{ target_.x, target_.y};
    grid_map[start.x][start.y] = "@";

    Node* current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(500);
    closedSet.reserve(500);
    openSet.push_back(new Node(start));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == finish) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    //CoordinateList path;
    while (current != nullptr) {
        path_.push_back(current->coordinates);
        grid_map[current->coordinates.x][current->coordinates.y] = "@"; 
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);
}

void AStar::Generator::changeRealPath(const CoordinateList& path_, PathList& robot_path_, double scale, Vec3d start_, Vec3d target_)
{
    robot_path_.clear();
    for (int i = 0; i < path_.size(); i++) {
        Vec3d new_path_;
        //new_path_.x = -0.01 * (1 / scale) * path_[i].x + 0.1 + 0.58;
        //new_path_.y = -0.01 * (1 / scale) * path_[i].y + 0.2;
        new_path_.x = -0.01 * (1 / scale) * path_[i].x + 0.68;
        new_path_.y = -0.01 * (1 / scale) * path_[i].y + 0.2;
        new_path_.z = 0.15;
        robot_path_.push_back(new_path_);
    }
    robot_path_[0] = start_;
    robot_path_.push_back(target_);
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete* it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
