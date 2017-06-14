#include <string>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>


void file2Cloud(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	if (!boost::filesystem::exists(file_path))
		return;
	std::ifstream t;
	t.open(file_path);
	std::string line;
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	while (t)
	{
		std::getline(t, line);
		if (boost::starts_with(line, std::string("# ")))
		{
			continue;
		}
		else
		{
			std::vector<std::string> strVec;
			boost::algorithm::split(strVec, line, boost::is_any_of(" "));
			if (strVec.size() < 4)
				continue;
			pcl::PointXYZ p;
			p.x = atof(strVec[1].c_str());
			p.y = atof(strVec[2].c_str());
			p.z = atof(strVec[3].c_str());
			cloud->points.push_back(p);
		}
	}
	t.close();
}

int main(int argc, char** argv)
{
	std::string file_name = "points3D.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	file2Cloud(file_name, cloud);
	pcl::io::savePLYFile("cloud.ply", *cloud);
}