#ifndef FILE_FUNCTIONS_H
#define FILE_FUNCTIONS_H

#include <string>
#include <vector>

// Returns a vector of all the files in a given directory.

void ReadDirectory(std::string        const &DirName, 
                   std::vector<std::string> &aFileNames);

// Returns the data and time in the following format.
// Year, Month, Day, Hour, Minute, Second
// YYYY-MM-DDTHH:MM:SS

std::string GetDateAndTime();

// Returns the file name from a full path string.

std::string GetFileName(std::string const &FileName);

// Returns the file extention of a file string.

void FindFileExtension(std::string const &FileName,
                       std::string       &Extension);

// Read until a ';' is found or return false if one is not found.

bool ReadFileLine(FILE        *pFile,
                  std::string &sFileLine);

// Advance the file pointer to the given string.

bool ReadToString(FILE              *pFile,
                  std::string const &sData);

#endif // FILE_FUNCTIONS_H
