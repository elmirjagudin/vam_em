#include "stdafx.h"

#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <share.h>

#include <string>

using namespace std;

#include "vam.h"

void
die(char *err_msg)
{
    printf("error: %s\n", err_msg);
    abort();
}

bool
raw_fname(int i, string &fname)
{
    fname = "E:\\RectifiedImgs\\s1\\img" + to_string(i) + ".raw";
    return _access(fname.c_str(), 0) != -1;
}

void
read_file(FILE *stream, void *buffer, size_t num_bytes)
{
    if (fread(buffer, num_bytes, 1, stream) != 1)
    {
        die("could not read all bytes");
    }
}

void
read_raw_files(vam_handle *vah)
{
    int i = 0;
    string fname;
    FILE *stream;
    int width;
    int height;

    while (raw_fname(i++, fname))
    {        
        if ((stream = _fsopen(fname.c_str(), "rb", _SH_DENYNO)) == NULL)
        {
            die("error opening file");
        }

        read_file(stream, &width, 4);
        read_file(stream, &height, 4);

        void *pixels = malloc(width *  height * 4);
        read_file(stream, pixels, width *  height * 4);

        vam_process_frame(vah, pixels);

        free(pixels);
        fclose(stream);
    }
}

void *
read_raw_file(string fname)
{
    FILE *stream;
    int width;
    int height;

    if ((stream = _fsopen(fname.c_str(), "rb", _SH_DENYNO)) == NULL)
    {
        die("error opening file");
    }

    read_file(stream, &width, 4);
    read_file(stream, &height, 4);

    void *pixels = malloc(width *  height * 4);
    read_file(stream, pixels, width *  height * 4);

    fclose(stream);

    return pixels;
}


void
main()
{
    read_raw_files(vam_init(0, 0));
}
