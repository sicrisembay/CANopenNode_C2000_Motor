/*
 * (C) Copyright 2021
 * Sicris Rey Embay, sicris.embay@gmail.com
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LEN      (256)
#define MAX_FILEPATH_LEN  (256)
#define MAX_FILENAME_LEN  (64)

#define DEBUG_LOG         (0)

static void remove_char(char* str, char c)
{
    if(str == NULL) {
        printf("%s : Null string argument\n", "remove_char");
        exit(-1);
    }
    char *pr = str, *pw = str;
    while (*pr) {
        *pw = *pr++;
        pw += (*pw != c);
    }
    *pw = '\0';
}

static int find_last_char_pos(char * str, char c)
{
    if(str == NULL) {
        printf("%s : Null string argument\n", "find_last_char_pos");
        exit(-1);
    }
    int pos = 0;
    int len = strlen(str);
#if (DEBUG_LOG == 1)
    printf("len: %d\n", len);
#endif
    int i = 0;
    for(i = 0; i < len; i++) {
        if(str[i] == c) {
            pos = i;
        }
    }
    return pos;
}

static void get_path(char * path, char * name, char * fullpath)
{
    if((path == NULL) || (name == NULL) || (fullpath == NULL)) {
        printf("%s : Null string argument\n", "get_path");
        exit(-1);
    }

    int len = strlen(fullpath);
    int sep_idx = find_last_char_pos(fullpath, '/') + 1;
#if (DEBUG_LOG == 1)
     printf("last idx: %d\n", sep_idx);
#endif
    if(sep_idx > MAX_FILEPATH_LEN) {
        printf("Exceeded Maximum Path length!\n");
        exit(-1);
    }
    memcpy(path, fullpath, sep_idx);
    path[sep_idx] = '\0';
    if((len-sep_idx) > MAX_FILENAME_LEN) {
        printf("Exceeded Maximum name length!\n");
        exit(-1);
    }
    memcpy(name, fullpath + sep_idx, (len - sep_idx));
    name[len-sep_idx] = '\0';
}

int main (int argc, char **argv)
{
    int c;
    char strline[MAX_LINE_LEN];
    char file_path[MAX_FILEPATH_LEN];
    char file_name[MAX_FILENAME_LEN];
    const char needle[] = "CONFIG_FLOAT_VALUE";
    char *input = argv[1];
    char output[MAX_FILEPATH_LEN + MAX_FILENAME_LEN];
    FILE *input_file;
    FILE *output_file;

    get_path(file_path, file_name, input);
#if (DEBUG_LOG == 1)
    printf("path: %s\n", file_path);
    printf("name: %s\n", file_name);
#endif
    memset(output, 0, sizeof(output));
    strcpy(output, file_path);
    strcat(output, "autoconf_post.h");
#if (DEBUG_LOG == 1)
    printf("output file: %s\n", output);
#endif
    output_file = fopen(output, "w+");
    if(output_file == 0) {
        perror("Canot open output file\n");
        exit(-1);
    }

    input_file = fopen(input, "r");

    if (input_file == 0) {
        fclose(output_file);
        //fopen returns 0, the NULL pointer, on failure
        perror("Canot open input file\n");
        exit(-1);
    } else {
        printf("Processing %s", input);
        while(fgets(strline, MAX_LINE_LEN, input_file) != NULL) {
            if(strstr(strline, needle) != NULL) {
                remove_char(strline, '"');
            }
#if (DEBUG_LOG == 1)
            printf("%s", strline);
#endif
            fputs(strline, output_file);
        }
        printf("\n");
    }

    fclose(output_file);
    fclose(input_file);

    printf("...Done\n");

    return 0;
}
