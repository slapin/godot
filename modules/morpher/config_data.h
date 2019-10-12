#ifndef CONFIG_DATA_H
#define CONFIG_DATA_H
#include <core/reference.h>
#include <scene/resources/mesh.h>

/* config data */

class ConfigData {
    Dictionary config;
    ConfigData();
public:
    static ConfigData *get_singleton();
    const Dictionary &get()
    {
	    return config;
    }
};

class AccessoryData {
    Dictionary accessory;
    AccessoryData();
public:
    PoolVector<Dictionary> get_matching_entries(const String &gender,
        const String &category, const String &match) const
    {
        const Dictionary &cat = accessory[gender];
        const Dictionary &items = cat[category];
        PoolVector<Dictionary> ret;
        for (const Variant *key = items.next(NULL);
            key; key = items.next(key)) {
            const String k = *key;
            if (k.match(match)) {
                const Dictionary &item = items[*key];
                ret.push_back(item);
            }
        }
        return ret;
    }
    Dictionary get_entry(const String &gender,
        const String &category, const String &name) const
    {
        const Dictionary &cat = accessory[gender];
        const Dictionary &items = cat[category];
        return items[name];
    }
    Ref<ArrayMesh> get_mesh(const Dictionary &entry) const;
    static AccessoryData *get_singleton();
};

#endif
