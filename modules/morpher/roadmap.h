#ifdef ROADMAP_H
#define ROADMAP_H

struct vertex {
	Vector2 pos;
	Vector2i chunk;
};

class vertex_grid: public Reference {
	GDCLASS(vertex_grid, Node);	
	OBJ_CATEGORY("slapin");
protected:
	int grid_size;
	HashMap<Vector2i, PoolArray<int> > grid;
public:
	vertex_grid()
	{
		grid_size = 32;
	}
	void set_grid_size(int size)
	{
		grid_size = size;
	}
	int get_grid_size() const
	{
		return grid_size;
	}
	void add(const Vector2 &pos, int id)
	{
		Vector2i key((int)(pos.x / (float)grid_size), (int)(pos.y / (float)grid_size));
		grid[key].push_back(pos);
	}
	void remove(int id)
	{
		const Vector2i *key = NULL;
		PoolVector<Vector2i> removed;
		while ((key = grid.next(key))) {
			for (int i = 0; i < grid[*key].size(); i++) {
				if (grid[*key][i] == id)
					removes.push_back(*key);
			}
		}

	}
}
class roadmap_ : public Reference {
	GDCLASS(roadmap_, Node);	
	OBJ_CATEGORY("slapin");
protected:
	PoolVector<Vector2> vertices;
	HashMap<Vector2, int> vertex2id;
	HashMap<int, Vector2> id2vertex;
	HashMap<int, Vector<int> > neighbors;
	HashMap<int, bool> vertex_seed;
	HashMap<int, int> vertex_type;
	static void _bind_methods();
public:
	int add_vertex(const Vector2 &vertex, bool seed, int type);
	{
		int id = vertices.size();
		vertices.push_back(vertex);
		vertex2id[vertex] = id;
		id2vertex[id] = vertex;
		vertex_type[id] = type;
		vertex_seed[id] = seed;
		return id;
	}
	bool has_neighbor(int pt, int n)
	{
		if (neighbors[pt].find(n) < 0)
			return false;
		return true;
	}
	int add_neighbor(int pt, int n)
	{
		if (neighbors[pt].find(n) < 0)
			neighbors[pt].push_back(n);
		if (neighbors[n].find(pt) < 0)
			neighbors[n].push_back(pt);
	}
	Vector<Vector2> front;
	PoolVector<Vector2> check_suggestion(int 
};
#endif
