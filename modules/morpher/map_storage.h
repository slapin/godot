#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <core/reference.h>
#include <core/image.h>
#include <cassert>

class MapStorage {
	static const long MAX_BUF = 52 * 1024 * 1024;
	struct datablock {
		String name;
		float minp[3], maxp[3], min_normal[3], max_normal[3];
		String helper;
		int width;
		int height;
		int format;
		int dec_size;
		uint8_t *buf;
		int buf_size;
		int width_normal;
		int height_normal;
		int format_normal;
		int dec_size_normal;
		uint8_t *buf_normal;
		int buf_normal_size;
	};
	uint8_t *buffer;
	unsigned long pos;
	HashMap<String, struct datablock *> data;
	Dictionary config;
	void load();
	MapStorage();
	uint8_t *allocate(unsigned long size) {
		assert(pos + size < MAX_BUF);
		uint8_t *ret = &buffer[pos];
		pos += (size + 32) & (~32UL);
		return ret;
	}

public:
	const Dictionary &get_config() const {
		return config;
	}
	PoolVector<String> get_list() const {
		PoolVector<String> ret;
		for (const String *key = data.next(NULL);
				key; key = data.next(key)) {
			ret.push_back(*key);
		}
		return ret;
	}
	Ref<Image> get_image(const String &name) const;
	Ref<Image> get_normal_image(const String &name) const;
	PoolVector<float> get_minmax(const String &shape_name);
	static MapStorage *get_singleton();
	~MapStorage();
};

#endif
