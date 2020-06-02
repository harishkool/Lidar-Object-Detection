#pragma once
#ifndef RANSAC3D_FILTER_H_
#define RANSAC3D_FILTER_H_

#include <unordered_set>

template<typename PointT>
inline std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations m
	int rand1, rand2;
	PointT p1, p2, p3, p4;
	float i, j, k, d, val;
	for(int iter=0; iter < maxIterations; iter++){

		std::unordered_set<int> inliers;

		while(inliers.size()<3){
			rand1 = (rand()%cloud->points.size());
			inliers.insert(rand1);
		}
		auto itr = inliers.begin();
		
		p1 = cloud->points[*itr];
		itr++;
		p2 = cloud->points[*itr];
		itr++;
		p3 = cloud->points[*itr];

		i = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
		j = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
		k = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x); 

		d = -(i*p1.x + j*p1.y + k*p1.z);
		for(int p=0; p < cloud->points.size(); p++){
			if(inliers.count(p)>0)
				continue;	
			p4 = cloud->points[p];
			val = fabs(i*p4.x + j*p4.y + k*p4.z + d) / (sqrt(i*i + j*j + k*k));
			if(val <= distanceTol){
				inliers.insert(p);
			}
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}

	return inliersResult;

}

#endif