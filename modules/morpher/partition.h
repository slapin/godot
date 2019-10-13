#ifdef TRIANGLE_H
#define TRIANGLE_H
#endif
class PartitionGrid : public Reference {
	GDCLASS(PartitionGrid, Reference)
protected:
	struct triangle {
		int id;
		int points[3];
	} entry entries[5 * 5 * 5];

public:
	PartitionGrid() {
	}
	~PartitionGrid() {
	}
};
