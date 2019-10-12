#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <core/reference.h>

class MapStorage {
	struct datablock {
		String name;
		float minp[3], maxp[3], min_normal[3], max_normal[3];
		String helper;
		int width;
		int height;
		int format;
		int dec_size;
		PoolVector<uint8_t> buf;
		int width_normal;
		int height_normal;
		int format_normal;
		int dec_size_normal;
		PoolVector<uint8_t> buf_normal;
	};
	HashMap<String, struct datablock> data;
	Dictionary config;
	void load();
	MapStorage();
public:
	const Dictionary &get_config() const {
		return config;
	}
	PoolVector<String> get_list() const
	{
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
	void remove_map(const String &name)
	{
		data.erase(name);
	}
	static MapStorage *get_singleton()
	{
		static MapStorage ms;
		return &ms;
	}
};

#endif