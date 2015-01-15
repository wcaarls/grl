#ifndef ANN_kd_tree2_H
#define ANN_kd_tree2_H

class ANNkd_node2{						// generic kd-tree node (empty shell)
public:
};

class ANNkd_leaf2: public ANNkd_node2		// leaf node for kd-tree
{
	int					n_pts;			// no. points in bucket
	ANNidxArray			bkt;			// bucket of points
public:

};

class ANNkd_split2 : public ANNkd_node2	// splitting node of a kd-tree
{
	int					cut_dim;		// dim orthogonal to cutting plane
	ANNcoord			cut_val;		// location of cutting plane
	ANNcoord			cd_bnds[2];		// lower and upper bounds of
										// rectangle along cut_dim
	ANNkd_ptr			child[2];		// left and right children
public:
};

#endif
