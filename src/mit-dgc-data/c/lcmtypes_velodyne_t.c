/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by LCM
 **/

#include <string.h>
#include "lcmtypes_velodyne_t.h"

static int __lcmtypes_velodyne_t_hash_computed;
static int64_t __lcmtypes_velodyne_t_hash;
 
int64_t __lcmtypes_velodyne_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __lcmtypes_velodyne_t_get_hash)
            return 0;
 
    const __lcm_hash_ptr cp = { p, (void*)__lcmtypes_velodyne_t_get_hash };
    (void) cp;
 
    int64_t hash = 0xb6befe08f6d416d3LL
         + __int64_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __byte_hash_recursive(&cp)
        ;
 
    return (hash<<1) + ((hash>>63)&1);
}
 
int64_t __lcmtypes_velodyne_t_get_hash(void)
{
    if (!__lcmtypes_velodyne_t_hash_computed) {
        __lcmtypes_velodyne_t_hash = __lcmtypes_velodyne_t_hash_recursive(NULL);
        __lcmtypes_velodyne_t_hash_computed = 1;
    }
 
    return __lcmtypes_velodyne_t_hash;
}
 
int __lcmtypes_velodyne_t_encode_array(void *buf, int offset, int maxlen, const lcmtypes_velodyne_t *p, int elements)
{
    int pos = 0, thislen, element;
 
    for (element = 0; element < elements; element++) {
 
        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].datalen), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __byte_encode_array(buf, offset + pos, maxlen - pos, p[element].data, p[element].datalen);
        if (thislen < 0) return thislen; else pos += thislen;
 
    }
    return pos;
}
 
int lcmtypes_velodyne_t_encode(void *buf, int offset, int maxlen, const lcmtypes_velodyne_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmtypes_velodyne_t_get_hash();
 
    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    thislen = __lcmtypes_velodyne_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    return pos;
}
 
int __lcmtypes_velodyne_t_encoded_array_size(const lcmtypes_velodyne_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {
 
        size += __int64_t_encoded_array_size(&(p[element].utime), 1);
 
        size += __int32_t_encoded_array_size(&(p[element].datalen), 1);
 
        size += __byte_encoded_array_size(p[element].data, p[element].datalen);
 
    }
    return size;
}
 
int lcmtypes_velodyne_t_encoded_size(const lcmtypes_velodyne_t *p)
{
    return 8 + __lcmtypes_velodyne_t_encoded_array_size(p, 1);
}
 
int __lcmtypes_velodyne_t_decode_array(const void *buf, int offset, int maxlen, lcmtypes_velodyne_t *p, int elements)
{
    int pos = 0, thislen, element;
 
    for (element = 0; element < elements; element++) {
 
        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].datalen), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        p[element].data = (uint8_t*) lcm_malloc(sizeof(uint8_t) * p[element].datalen);
        thislen = __byte_decode_array(buf, offset + pos, maxlen - pos, p[element].data, p[element].datalen);
        if (thislen < 0) return thislen; else pos += thislen;
 
    }
    return pos;
}
 
int __lcmtypes_velodyne_t_decode_array_cleanup(lcmtypes_velodyne_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {
 
        __int64_t_decode_array_cleanup(&(p[element].utime), 1);
 
        __int32_t_decode_array_cleanup(&(p[element].datalen), 1);
 
        __byte_decode_array_cleanup(p[element].data, p[element].datalen);
        if (p[element].data) free(p[element].data);
 
    }
    return 0;
}
 
int lcmtypes_velodyne_t_decode(const void *buf, int offset, int maxlen, lcmtypes_velodyne_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmtypes_velodyne_t_get_hash();
 
    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;
 
    thislen = __lcmtypes_velodyne_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    return pos;
}
 
int lcmtypes_velodyne_t_decode_cleanup(lcmtypes_velodyne_t *p)
{
    return __lcmtypes_velodyne_t_decode_array_cleanup(p, 1);
}
 
int __lcmtypes_velodyne_t_clone_array(const lcmtypes_velodyne_t *p, lcmtypes_velodyne_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {
 
        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);
 
        __int32_t_clone_array(&(p[element].datalen), &(q[element].datalen), 1);
 
        q[element].data = (uint8_t*) lcm_malloc(sizeof(uint8_t) * q[element].datalen);
        __byte_clone_array(p[element].data, q[element].data, p[element].datalen);
 
    }
    return 0;
}
 
lcmtypes_velodyne_t *lcmtypes_velodyne_t_copy(const lcmtypes_velodyne_t *p)
{
    lcmtypes_velodyne_t *q = (lcmtypes_velodyne_t*) malloc(sizeof(lcmtypes_velodyne_t));
    __lcmtypes_velodyne_t_clone_array(p, q, 1);
    return q;
}
 
void lcmtypes_velodyne_t_destroy(lcmtypes_velodyne_t *p)
{
    __lcmtypes_velodyne_t_decode_array_cleanup(p, 1);
    free(p);
}
