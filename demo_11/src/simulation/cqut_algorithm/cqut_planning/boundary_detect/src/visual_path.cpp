#include "boundary_detect/visual_path.hpp"
#include <rclcpp/rclcpp.hpp>

namespace FSD 
{

void visual(cv::Subdiv2D coneSet, SearchTree Path, 
        cqut_msg::msg::Map boundaryDetections, 
        std::vector<PathPoint> BestPath,
        visualization_msgs::msg::Marker &visualTriangles, 
        visualization_msgs::msg::MarkerArray &visualTree, 
        visualization_msgs::msg::MarkerArray &visualBoundary,
        visualization_msgs::msg::Marker &visualPath) 
{        
        // visualization delaunay triangles

        std::vector<cv::Vec4f> edges;

		coneSet.getEdgeList(edges);

		// the virtual triangle three edges
		cv::Point2f outer_vtx[3];
		for (int i = 0; i < 3; i++)
		{
			outer_vtx[i] = coneSet.getVertex(i + 1);
		}

		visualization_msgs::msg::Marker line_list;
		line_list.header.frame_id = "/base_link";
		line_list.header.stamp = rclcpp::Clock().now();
		line_list.ns = "points_and_lines";
		line_list.action = visualization_msgs::msg::Marker::ADD;
		line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
		line_list.scale.x = 0.1;
		line_list.color.g = 1.0;
		line_list.color.a = 1.0;

		geometry_msgs::msg::Point p1, p2;

		for (int i = 0; i < edges.size(); i++)
		{
			// abandon the far points of virtual triangle
			if (edges[i][0] == outer_vtx[0].x && edges[i][1] == outer_vtx[0].y ||
				edges[i][0] == outer_vtx[1].x && edges[i][1] == outer_vtx[1].y ||
				edges[i][0] == outer_vtx[2].x && edges[i][1] == outer_vtx[2].y ||
				edges[i][2] == outer_vtx[0].x && edges[i][3] == outer_vtx[0].y ||
				edges[i][2] == outer_vtx[1].x && edges[i][3] == outer_vtx[1].y ||
				edges[i][2] == outer_vtx[2].x && edges[i][3] == outer_vtx[2].y)
				continue;
			p1.x = edges[i][0];
			p1.y = edges[i][1];
			p1.z = 0;
			p2.x = edges[i][2];
			p2.y = edges[i][3];
			p2.z = 0;
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
		visualTriangles = line_list;
		
        // visualization Tree Path
        std::vector<std::vector<PathPoint>> tree_path;
        std::vector<SearchTree*> next;
        std::vector<SearchTree*> tmp;

        next.push_back(&Path);

        while(next.size() != 0) 
        {
            tmp.clear();
            for(auto &iter :next) 
            {
                if(iter->next.size() == 0) 
                {
                    tree_path.push_back(iter->history);
                    //print_path(iter.history);
                }
                else 
                {
                    for(auto &it:iter->next)
                        tmp.push_back(&it);
                }
            }
            next = tmp;
        }
        
        int path_num = 0;
        visualTree.markers.clear();
        for(const auto &iter: tree_path) 
        {
            visualization_msgs::msg::Marker line_list;
            line_list.header.frame_id = "/base_link";
            line_list.header.stamp = rclcpp::Clock().now();
            line_list.ns = "path" + std::to_string(path_num);
            line_list.action = visualization_msgs::msg::Marker::ADD;
            line_list.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_list.scale.x = 0.1;
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;
            geometry_msgs::msg::Point p;
            for(const auto &it: iter) 
            {
                p.x = it.x;
                p.y = it.y;
                line_list.points.push_back(p);
            }
            visualTree.markers.push_back(line_list);
            path_num++;
        }

        // visualization Boundary
        visualBoundary.markers.clear();
        visualization_msgs::msg::Marker boundary_red;
        boundary_red.header.frame_id = "/base_link";
        boundary_red.header.stamp = rclcpp::Clock().now();
        boundary_red.ns = "red";
        boundary_red.action = visualization_msgs::msg::Marker::ADD;
        boundary_red.type = visualization_msgs::msg::Marker::LINE_STRIP;
        boundary_red.scale.x = 0.8;
        boundary_red.color.r = 1.0;
        boundary_red.color.a = 1.0;
        geometry_msgs::msg::Point p;
        for(const auto &iter: boundaryDetections.cone_red) 
        {
            p.x = iter.position.x;
            p.y = iter.position.y;
            boundary_red.points.push_back(p);
        }

        visualization_msgs::msg::Marker boundary_blue;
        boundary_blue.header.frame_id = "/base_link";
        boundary_blue.header.stamp = rclcpp::Clock().now();
        boundary_blue.ns = "blue";
        boundary_blue.action = visualization_msgs::msg::Marker::ADD;
        boundary_blue.type = visualization_msgs::msg::Marker::LINE_STRIP;
        boundary_blue.scale.x = 0.8;
        boundary_blue.color.b = 1.0;
        boundary_blue.color.a = 1.0;
        for(const auto &iter: boundaryDetections.cone_blue) 
        {
            p.x = iter.position.x;
            p.y = iter.position.y;
            boundary_blue.points.push_back(p);
        }

        visualBoundary.markers.push_back(boundary_red);
        visualBoundary.markers.push_back(boundary_blue);

        // visualization BestPath
        visualPath.points.clear();
        visualPath.header.frame_id = "/base_link";
        visualPath.header.stamp = rclcpp::Clock().now();
        visualPath.ns = "best_path";
        visualPath.action = visualization_msgs::msg::Marker::ADD;
        visualPath.type = visualization_msgs::msg::Marker::LINE_STRIP;
        visualPath.scale.x = 0.8;
        visualPath.color.r = 0;
        visualPath.color.g = 0;
        visualPath.color.b = 0;
        visualPath.color.a = 1.0;
        for(const auto &iter: BestPath) 
        {
            p.x = iter.x;
            p.y = iter.y;
            visualPath.points.push_back(p);
        }

    }
}