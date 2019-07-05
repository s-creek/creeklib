#ifndef KD_TREE_CLASS_HPP
#define KD_TREE_CLASS_HPP

//----------------------------------------------------------------------------------------
// <kd-tree>
//
// ref.1
//   https://hope.c.fun.ac.jp/course/view.php?id=373
//   https://hope.c.fun.ac.jp/pluginfile.php/33640/mod_resource/content/1/2014-Kd-tree%E3%81%A8%E6%9C%80%E8%BF%91%E5%82%8D%E6%8E%A2%E7%B4%A2.pdf?forcedownload=1
//
// ref.2
//   http://atkg.hatenablog.com/entry/2016/12/18/002353
//
//----------------------------------------------------------------------------------------


#include <vector>
#include <memory>    // shared_ptr
#include <algorithm> // nth_element
#include <numeric>   // iota
#include <limits>    // limit
//#include <queue>     // priority_queue

#include <iostream>  // debug


// PointType
//  size()
//  operator []

template<class PointType>
class KdTree
{
	//----------------------------------------------------------------------------------------
	// Node class (tree\‘¢‚ğ\¬‚·‚éƒm[ƒhƒNƒ‰ƒX)
	//----------------------------------------------------------------------------------------

public:
	class Node;
	using NodePtr = std::shared_ptr<Node>;


	class Node
	{
	public:
		Node() : m_index(0), m_axis(0) {}
		Node(std::size_t index, std::size_t axis) {
			m_index = index;
			m_axis = axis;
		}

		const std::size_t index() const { return m_index; }
		const std::size_t axis() const { return m_axis; }

		NodePtr& child(std::size_t lh) { return m_child[lh]; }
		const NodePtr& child(std::size_t lh) const { return m_child[lh]; }

		NodePtr& lo() { return m_child[0]; }
		NodePtr& hi() { return m_child[1]; }

		const NodePtr& lo() const { return m_child[0]; }
		const NodePtr& hi() const { return m_child[1]; }


	private:
		std::size_t m_index, m_axis;
		NodePtr m_child[2];
	};




	//----------------------------------------------------------------------------------------
	// BoundedPriorityQueue class
	//----------------------------------------------------------------------------------------

	//
	// Container = std::vector or std::deque
	//
	template<class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type> >
	class BoundedPriorityQueue : public Container
	{
	public:
		BoundedPriorityQueue() : Container() {}

		void push(const T& val) {
			typename Container::iterator it = std::find_if(this->begin(), this->end(), [&](const T& p) { return Compare()(val, p); });
			this->insert(it, val);
		}

		T& top() { return this->front(); }
		const T& top() const { return this->front(); }

		void pop() { this->erase(this->begin()); }
	};

	// piar <index, distance>
	using KnnNode = std::pair<std::size_t, double>;

	struct KnnCompare
	{
		bool operator()(const KnnNode& l, const KnnNode& r) {
			return l.second < r.second;
		}
	};

	using KnnQueue = BoundedPriorityQueue<KnnNode, std::vector<KnnNode>, KnnCompare>;  // ©ì‚Ì•û‚ª‘¬‚©‚Á‚½EEE“ä
	//using KnnQueue = std::priority_queue<KnnNode, std::vector<KnnNode>, KnnCompare>;




	//----------------------------------------------------------------------------------------
	// kd-tree class
	//----------------------------------------------------------------------------------------

public:
	KdTree() {}
	
	template <template <class T, class A = std::allocator<T> > class Container>
	KdTree(const Container<PointType> &points, const std::size_t leaf_size = 20) { this->build(points, leaf_size); }


	const std::size_t dim() const { return m_dim; }
	const NodePtr root() const { return m_root; }
	const PointType& point(std::size_t index) const { return m_points.at(index); }
	const std::vector<PointType>& points() const { return m_points; }


	//
	// kd-tree‚Ì\’z
	//
	template <template <class T, class A = std::allocator<T> > class Container>
	void build(const Container<PointType> &points, const std::size_t leaf_size = 20);


	//
	// Å‹ß–T’Tõ (nearest neighbor search)
	//
	std::size_t nnSearch(const PointType& query);
	std::size_t nnSearch(const PointType& query, double& dist);


	//
	// k‹ß–T’Tõ (k-nearest neighbor search)
	// ‹ß‚¢‡‚ÉkŒÂ‚Ìƒm[ƒh‚ğ’Tõ
	//
	void knnSearch(const PointType& query, const std::size_t k, std::vector<std::size_t>& indices);

	template<class ValueType>
	void knnSearch(const PointType& query, const std::size_t k, std::vector<std::size_t>& indices, std::vector<ValueType>& distances);


	//
	// ”¼Œa“à‚ÉŠÜ‚Ü‚ê‚éƒm[ƒh’Tõ (radius search)
	//
	void radiusSearch(const PointType& query, const double radius, std::vector<std::size_t>& indices, bool sort = false);

	template<class ValueType>
	void radiusSearch(const PointType& query, const double radius, std::vector<std::size_t>& indices, std::vector<ValueType>& distances, bool sort = false);


	//
	// Še²ŠÔ‹——£‚ª}range“à‚É‚ ‚éƒm[ƒh’Tõ (2ŸŒ³‚È‚ç³•ûA3ŸŒ³‚È‚ç—§•û)
	//
	void rangeSearch(const PointType& query, const double range, std::vector<std::size_t>& indices, bool sort = false);

	template<class ValueType>
	void rangeSearch(const PointType& query, const double range, std::vector<std::size_t>& indices, std::vector<ValueType>& distances, bool sort = false);


private:
	double distance(const PointType& l, const PointType& r);

	// kd-tree\’z
	NodePtr buildRecursive(std::vector<std::size_t>& indices, const int lo, const int hi, const std::size_t k, const std::size_t leaf_size);


	// Å‹ß–T’Tõ (nearest neighbor search)
	void nnSearchRecursive(const NodePtr node, const PointType& query, std::size_t& guess, double& dist);


	// k‹ß–T’Tõ (k-nearest neighbor search)
	void knnSearchRecursive(const NodePtr node, const PointType& query, const std::size_t k, KnnQueue& queue);


	// ”¼Œa“à‚ÉŠÜ‚Ü‚ê‚éƒm[ƒh’Tõ(radius search)
	void radiusSearchRecursive(const NodePtr node, const PointType& query, const double radius, std::vector<std::size_t>& indices);

	template<class ValueType>
	void radiusSearchRecursive(const NodePtr node, const PointType& query, const double radius, std::vector<std::size_t>& indices, std::vector<ValueType>& distances);

	void radiusSearchRecursiveSort(const NodePtr node, const PointType& query, const double radius, KnnQueue& queue);


	// Še²ŠÔ‹——£‚ª}range“à‚É‚ ‚éƒm[ƒh’Tõ (2ŸŒ³‚È‚ç³•ûA3ŸŒ³‚È‚ç—§•û)
	void rangeSearchRecursive(const NodePtr node, const PointType& query, const double range, std::vector<std::size_t>& indices);

	template<class ValueType>
	void rangeSearchRecursive(const NodePtr node, const PointType& query, const double range, std::vector<std::size_t>& indices, std::vector<ValueType>& distances);

	void rangeSearchRecursiveSort(const NodePtr node, const PointType& query, const double range, KnnQueue& queue);



	// PointType‚ÌŸŒ³
	std::size_t m_dim;

	// tree\‘¢‚Ìª
	NodePtr m_root;

	// “ü—Íƒf[ƒ^‚ÌƒRƒs[
	std::vector<PointType> m_points;
};




//----------------------------------------------------------------------------------------
// À‘•
//----------------------------------------------------------------------------------------

template<class PointType>
double KdTree<PointType>::distance(const PointType& l, const PointType& r)
{
	double square_sum(0.0);
	for (std::size_t i = 0; i < l.size(); i++) {
        double dl = (double)l[i];
        double dr = (double)r[i];
		square_sum += (dl - dr) * (dl - dr);
	}
	return std::sqrt(square_sum);
}


//
// kd-tree‚Ìì¬
//
template<class PointType>
template <template <class T, class A = std::allocator<T> > class Container>
void KdTree<PointType>::build(const Container<PointType> &points, const std::size_t leaf_size)
{
	if (points.empty()) {
		return;
	}

	// reset data
	m_points.clear();
	m_root.reset();


	// set data
	m_dim = points.front().size();
	m_points.assign(points.begin(), points.end());

	std::vector<std::size_t> indices(points.size());
	std::iota(std::begin(indices), std::end(indices), 0);


	// build kd-tree
	m_root = buildRecursive(indices, 0, points.size(), 0, leaf_size);
}


template<class PointType>
typename KdTree<PointType>::NodePtr KdTree<PointType>::buildRecursive(std::vector<std::size_t>& indices, const int lo, const int hi, const std::size_t k, const std::size_t leaf_size)
{
	if (lo >= hi) {
		return nullptr;
	}
	if (k >= leaf_size) {
		std::cout << "leaf size error" << std::endl;
		return nullptr;
	}


	// ’†‰›’l‚Å•ªŠ„
	int mid((hi + lo) / 2);
	std::size_t axis = k % m_dim;

	std::nth_element(indices.begin() + lo, indices.begin() + mid, indices.begin() + hi,
		[&](std::size_t left, std::size_t right) { return m_points[left][axis] < m_points[right][axis]; });


	// •ªŠ„ƒm[ƒhì¬
	NodePtr node(std::make_shared<Node>(indices[mid], axis));

	node->lo() = buildRecursive(indices, lo, mid, k + 1, leaf_size);
	node->hi() = buildRecursive(indices, mid + 1, hi, k + 1, leaf_size);

	return node;
}


//
// Å‹ß–T’Tõ (nearest neighbor search)
//
template<class PointType>
std::size_t KdTree<PointType>::nnSearch(const PointType& query)
{
	double dist;
	return nnSearch(query, dist);
}


template<class PointType>
std::size_t KdTree<PointType>::nnSearch(const PointType& query, double& dist)
{
	std::size_t guess(0);

	dist = std::numeric_limits<double>::max();
	nnSearchRecursive(m_root, query, guess, dist);

	return guess;
}


template<class PointType>
void KdTree<PointType>::nnSearchRecursive(const NodePtr node, const PointType& query, std::size_t& guess, double& dist)
{
	if (!node) {
		return;
	}

	// query‚ªŠÜ‚Ü‚ê‚é—Ìˆæ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	nnSearchRecursive(node->child(lh), query, guess, dist);


	// ‹——£ŒvZ
	double q_dist(distance(query, train));

	if (q_dist < dist) {
		guess = node->index();
		dist = q_dist;
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) < dist) {
		nnSearchRecursive(node->child(1 - lh), query, guess, dist);
	}
}


//
// k‹ß–T’Tõ (k-nearest neighbor search)
//
template<class PointType>
void KdTree<PointType>::knnSearch(const PointType& query, const std::size_t k, std::vector<std::size_t>& indices)
{
	indices.clear();

	KnnQueue queue;

	knnSearchRecursive(m_root, query, k, queue);

	std::size_t n(std::min(k, queue.size()));
	indices.resize(n);
	for (std::size_t i = 0; i < n; i++) {
		indices.at(i) = queue.at(i).first;
	}
}


template<class PointType>
template<class ValueType>
void KdTree<PointType>::knnSearch(const PointType& query, const std::size_t k, std::vector<std::size_t>& indices, std::vector<ValueType>& distances)
{
	indices.clear();
	distances.clear();

	KnnQueue queue;

	knnSearchRecursive(m_root, query, k, queue);

	std::size_t n(std::min(k, queue.size()));
	indices.resize(n);
	distances.resize(n);
	for (std::size_t i = 0; i < n; i++) {
		indices.at(i) = queue.at(i).first;
		distances.at(i) = (ValueType)queue.at(i).second;
	}
}


template<class PointType>
void KdTree<PointType>::knnSearchRecursive(const NodePtr node, const PointType& query, const std::size_t k, KnnQueue& queue)
{
	if (!node) {
		return;
	}


	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	knnSearchRecursive(node->child(lh), query, k, queue);


	// ‹——£ŒvZ
	double q_dist(distance(query, train));
	queue.push(std::make_pair(node->index(), q_dist));


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) < queue.back().second || queue.size() < k) {
		knnSearchRecursive(node->child(1 - lh), query, k, queue);
	}
}


//
// ”¼Œa“à‚ÉŠÜ‚Ü‚ê‚é‹ß–T’Tõ (radius search)
//
template<class PointType>
void KdTree<PointType>::radiusSearch(const PointType& query, const double radius, std::vector<std::size_t>& indices, bool sort)
{
	indices.clear();

	if (!sort) {
		radiusSearchRecursive(m_root, query, radius, indices);
	}
	else {
		KnnQueue queue;
		radiusSearchRecursiveSort(m_root, query, radius, queue);

		indices.resize(queue.size());
		for (std::size_t i = 0; i < queue.size(); i++) {
			indices.at(i) = queue.at(i).first;
		}
	}
}


template<class PointType>
template<class ValueType>
void KdTree<PointType>::radiusSearch(const PointType& query, const double radius, std::vector<std::size_t>& indices, std::vector<ValueType>& distances, bool sort)
{
	indices.clear();
	distances.clear();

	if (!sort) {
		radiusSearchRecursive(m_root, query, radius, indices, distances);
	}
	else {
		KnnQueue queue;
		radiusSearchRecursiveSort(m_root, query, radius, queue);

		indices.resize(queue.size());
		distances.resize(queue.size());
		for (std::size_t i = 0; i < queue.size(); i++) {
			indices.at(i) = queue.at(i).first;
			distances.at(i) = (ValueType)queue.at(i).second;
		}
	}
}


template<class PointType>
void KdTree<PointType>::radiusSearchRecursive(const NodePtr node, const PointType& query, const double radius, std::vector<std::size_t>& indices)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	radiusSearchRecursive(node->child(lh), query, radius, indices);


	// ‹——£ŒvZ
	double q_dist(distance(query, train));
	if (q_dist <= radius) {
		indices.push_back(node->index());
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) <= radius) {
		radiusSearchRecursive(node->child(1 - lh), query, radius, indices);
	}
}


template<class PointType>
template<class ValueType>
void KdTree<PointType>::radiusSearchRecursive(const NodePtr node, const PointType& query, const double radius, std::vector<std::size_t>& indices, std::vector<ValueType>& distances)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	radiusSearchRecursive(node->child(lh), query, radius, indices, distances);


	// ‹——£ŒvZ
	double q_dist(distance(query, train));
	if (q_dist <= radius) {
		indices.push_back(node->index());
		distances.push_back((ValueType)q_dist);
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) <= radius) {
		radiusSearchRecursive(node->child(1 - lh), query, radius, indices, distances);
	}
}


template<class PointType>
void KdTree<PointType>::radiusSearchRecursiveSort(const NodePtr node, const PointType& query, const double radius, KnnQueue& queue)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	radiusSearchRecursiveSort(node->child(lh), query, radius, queue);


	// ‹——£ŒvZ
	double q_dist(distance(query, train));
	if (q_dist <= radius) {
		queue.push(std::make_pair(node->index(), q_dist));
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	double diff(std::fabs(query[axis] - train[axis]));
	if (diff <= radius) {
		radiusSearchRecursiveSort(node->child(1 - lh), query, radius, queue);
	}
}


//
// Še²ŠÔ‹——£‚ª}range“à‚É‚ ‚éƒm[ƒh’Tõ (2ŸŒ³‚È‚ç³•ûA3ŸŒ³‚È‚ç—§•û)
//
template<class PointType>
void KdTree<PointType>::rangeSearch(const PointType& query, const double range, std::vector<std::size_t>& indices, bool sort)
{
	indices.clear();

	if (!sort) {
		rangeSearchRecursive(m_root, query, range, indices);
	}
	else {
		KnnQueue queue;
		rangeSearchRecursiveSort(m_root, query, range, queue);

		indices.resize(queue.size());
		for (std::size_t i = 0; i < queue.size(); i++) {
			indices.at(i) = queue.at(i).first;
		}
	}
}


template<class PointType>
template<class ValueType>
void KdTree<PointType>::rangeSearch(const PointType& query, const double range, std::vector<std::size_t>& indices, std::vector<ValueType>& distances, bool sort)
{
	indices.clear();
	distances.clear();

	if (!sort) {
		rangeSearchRecursive(m_root, query, range, indices, distances);
	}
	else {
		KnnQueue queue;
		rangeSearchRecursiveSort(m_root, query, range, queue);

		indices.resize(queue.size());
		distances.resize(queue.size());
		for (std::size_t i = 0; i < queue.size(); i++) {
			indices.at(i) = queue.at(i).first;
			distances.at(i) = (ValueType)queue.at(i).second;
		}
	}
}


template<class PointType>
void KdTree<PointType>::rangeSearchRecursive(const NodePtr node, const PointType& query, const double range, std::vector<std::size_t>& indices)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	rangeSearchRecursive(node->child(lh), query, range, indices);


	// ‹——£ŒvZ
	bool inRange(true);
	for (std::size_t i = 0; i < m_dim; i++) {
		if (std::fabs(query[i] - train[i]) > range) {
			inRange = false;
			break;
		}
	}
	if (inRange) {
		indices.push_back(node->index());
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) <= range) {
		rangeSearchRecursive(node->child(1 - lh), query, range, indices);
	}
}


template<class PointType>
template<class ValueType>
void KdTree<PointType>::rangeSearchRecursive(const NodePtr node, const PointType& query, const double range, std::vector<std::size_t>& indices, std::vector<ValueType>& distances)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	rangeSearchRecursive(node->child(lh), query, range, indices, distances);


	// ‹——£ŒvZ
	bool inRange(true);
	for (std::size_t i = 0; i < m_dim; i++) {
		if (std::fabs(query[i] - train[i]) > range) {
			inRange = false;
			break;
		}
	}
	if (inRange) {
		indices.push_back(node->index());
		distances.push_back((ValueType)distance(query, train));
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) <= range) {
		rangeSearchRecursive(node->child(1 - lh), query, range, indices, distances);
	}
}


template<class PointType>
void KdTree<PointType>::rangeSearchRecursiveSort(const NodePtr node, const PointType& query, const double range, KnnQueue& queue)
{
	if (!node) {
		return;
	}

	// ‹ß–T‘¤‚ğ’Tõ
	const PointType& train = m_points.at(node->index());
	std::size_t axis = node->axis();
	std::size_t lh = query[axis] < train[axis] ? 0 : 1;
	rangeSearchRecursiveSort(node->child(lh), query, range, queue);


	// ‹——£ŒvZ
	bool inRange(true);
	for (std::size_t i = 0; i < m_dim; i++) {
		if (std::fabs(query[i] - train[i]) > range) {
			inRange = false;
			break;
		}
	}
	if (inRange) {
		queue.push(std::make_pair(node->index(), distance(query, train)));
	}


	// ”½‘Î‘¤‚Ì—Ìˆæ’Tõ
	if (std::fabs(query[axis] - train[axis]) <= range) {
		rangeSearchRecursiveSort(node->child(1 - lh), query, range, queue);
	}
}


#endif // !KD_TREE_CLASS_HPP
