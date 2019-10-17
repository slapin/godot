#include "map_storage.h"
#include <core/image.h>
#include <core/io/compression.h>
#include <core/io/json.h>
#include <core/os/file_access.h>
#include <core/resource.h>
#include <cassert>

MapStorage::MapStorage() :
		pos(0), buffer(NULL) {
	buffer = memnew_arr(uint8_t, MAX_BUF);
	printf("map_storage created\n");
	FileAccess *fd = FileAccess::open("res://characters/config.json", FileAccess::READ);
	assert(fd);
	String confdata = fd->get_as_utf8_string();
	fd->close();
	String err;
	int err_line;
	Variant adata;
	Error e = JSON::parse(confdata, adata, err, err_line);
	if (e != OK)
		printf("json parse error: %ls at line %d\n", err.c_str(), err_line);
	assert(e == OK);
	config = adata;
	printf("loading\n");
	load();
	printf("loading done\n");
}

PoolVector<float> MapStorage::get_minmax(const String &shape_name) {
	int i;
	PoolVector<float> minmax;
	minmax.resize(12);
	struct datablock *d = data[shape_name];
	for (i = 0; i < 3; i++)
		minmax.write()[i] = d->minp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 3] = d->maxp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 6] = d->min_normal[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 9] = d->max_normal[i];
	return minmax;
}
void MapStorage::load() {
	int i, j;
	float minp[3], maxp[3], min_normal[3], max_normal[3];
	int width, height, format;
	int nwidth, nheight, nformat;
	int dec_size, comp_size;
	assert(config.has("map_path"));
	const String &map_path = config["map_path"];
	FileAccess *fd = FileAccess::open(map_path, FileAccess::READ);
	if (!fd)
		return;
	int count = fd->get_32();
	for (j = 0; j < count; j++) {
		struct datablock *d =
			(struct datablock *) allocate(sizeof(struct datablock));
		String shape_name = fd->get_pascal_string();
		printf("loading shape: %ls\n", shape_name.c_str());
		for (i = 0; i < 3; i++)
			d->minp[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d->maxp[i] = fd->get_float();
		d->width = fd->get_32();
		d->height = fd->get_32();
		d->format = fd->get_32();
		d->dec_size = fd->get_32();
		comp_size = fd->get_32();
		d->buf = allocate(comp_size);
		d->buf_size = comp_size;
		fd->get_buffer(d->buf, comp_size);
		for (i = 0; i < 3; i++)
			d->min_normal[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d->max_normal[i] = fd->get_float();
		d->width_normal = fd->get_32();
		d->height_normal = fd->get_32();
		d->format_normal = fd->get_32();
		d->dec_size_normal = fd->get_32();
		comp_size = fd->get_32();
		d->buf_normal = allocate(comp_size);
		d->buf_normal_size = comp_size;
		fd->get_buffer(d->buf_normal, comp_size);
		data[shape_name] = d;
	}
}
MapStorage::~MapStorage()
{
	memdelete_arr(buffer);
}
Ref<Image> MapStorage::get_image(const String &name) const {
	assert(data.has(name));
	const struct datablock *d = data[name];
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(d->dec_size);
	Compression::decompress(imgdecbuf.write().ptr(), d->dec_size,
			d->buf, d->buf_size, Compression::MODE_FASTLZ);
	Ref<Image> img = memnew(Image);
	assert(img.ptr() != NULL);
	img->create(d->width, d->height,
			false, (Image::Format)d->format, imgdecbuf);
	return img;
}
Ref<Image> MapStorage::get_normal_image(const String &name) const {
	const struct datablock *d = data[name];
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(d->dec_size_normal);
	Compression::decompress(imgdecbuf.write().ptr(),
			d->dec_size_normal, d->buf_normal,
			d->buf_normal_size, Compression::MODE_FASTLZ);
	Ref<Image> img = memnew(Image);
	assert(img.ptr() != NULL);
	img->create(d->width_normal, d->height_normal,
			false, (Image::Format)d->format_normal, imgdecbuf);
	return img;
}
MapStorage *MapStorage::get_singleton() {
	static MapStorage ms;
	return &ms;
}
