#include <cassert>
#include <core/os/file_access.h>
#include <core/io/json.h>
#include <core/resource.h>
#include <core/image.h>
#include <core/io/compression.h>
#include "map_storage.h"

MapStorage::MapStorage()
{
	FileAccess *fd = FileAccess::open("res://characters/config.json", FileAccess::READ);
	assert(fd);
	String confdata = fd->get_as_utf8_string();
	fd->close();
	String err;
	int err_line;
	Variant adata;
	JSON::parse(confdata, adata, err, err_line);
	config = adata;
	load();
}

PoolVector<float> MapStorage::get_minmax(const String &shape_name)
{
	int i;
	PoolVector<float> minmax;
	minmax.resize(12);
	struct datablock d = data[shape_name];
	for (i = 0; i < 3; i++)
		minmax.write()[i] = d.minp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 3] = d.maxp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 6] = d.min_normal[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 9] = d.max_normal[i];
	return minmax;
}
void MapStorage::load()
{
	int i, j;
	float minp[3], maxp[3], min_normal[3], max_normal[3];
	int width, height, format;
	int nwidth, nheight, nformat;
	int dec_size, comp_size;
	assert(config.has("map_path"));
	const String &map_path = config["map_path"];
	FileAccess *fd = FileAccess::open(map_path, FileAccess::READ);
	int count = fd->get_32();
	for (j = 0; j < count; j ++) {
		struct datablock d;
		String shape_name = fd->get_pascal_string();
		printf("loading shape: %ls\n", shape_name.c_str());
		for (i = 0; i < 3; i++)
			d.minp[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d.maxp[i] = fd->get_float();
		d.width = fd->get_32();
		d.height = fd->get_32();
		d.format = fd->get_32();
		d.dec_size = fd->get_32();
		comp_size = fd->get_32();
		d.buf.resize(comp_size);
		fd->get_buffer(d.buf.write().ptr(), comp_size);
		for (i = 0; i < 3; i++)
			d.min_normal[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d.max_normal[i] = fd->get_float();
		d.width_normal = fd->get_32();
		d.height_normal = fd->get_32();
		d.format_normal = fd->get_32();
		d.dec_size_normal = fd->get_32();
		comp_size = fd->get_32();
		d.buf_normal.resize(comp_size);
		fd->get_buffer(d.buf_normal.write().ptr(), comp_size);
		data[shape_name] = d;
	}
}
Ref<Image> MapStorage::get_image(const String &name) const
{
    printf("get_image1 \"%ls\"\n", name.c_str());
    assert(data.has(name));
	const struct datablock &d = data[name];
    printf("get_image2\n");
	PoolVector<uint8_t> imgdecbuf;
    printf("get_image3\n");
	imgdecbuf.resize(d.dec_size);
    printf("get_image4\n");
	Compression::decompress(imgdecbuf.write().ptr(), d.dec_size, d.buf.read().ptr(), d.buf.size(), Compression::MODE_DEFLATE);
    printf("get_image5\n");
	Ref<Image> img = memnew(Image);
    printf("get_image6\n");
    assert(img.ptr() != NULL);
    printf("get_image7\n");
	img->create(d.width, d.height, false, (Image::Format)d.format, imgdecbuf);
    printf("get_image8\n");
	return img;
}
Ref<Image> MapStorage::get_normal_image(const String &name) const
{
	const struct datablock &d = data[name];
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(d.dec_size_normal);
	Compression::decompress(imgdecbuf.write().ptr(),
		d.dec_size_normal, d.buf_normal.read().ptr(),
		d.buf_normal.size(), Compression::MODE_DEFLATE);
	Ref<Image> img = memnew(Image);
    assert(img.ptr() != NULL);
	img->create(d.width_normal, d.height_normal, false, (Image::Format)d.format_normal, imgdecbuf);
	return img;
}

