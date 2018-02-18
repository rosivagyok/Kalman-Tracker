#ifndef XmlWriter_H
#define XmlWriter_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class XmlWriter {
public:
	bool open(const std::string);
	void close();
	bool exists(const std::string);
	void writeOpenTag(const std::string);
	void writeCloseTag();
	void writeStartElementTag(const std::string);
	void writeEndElementTag();
	void writeEndElementNULLTag();
	void writeEndElementBox();
	void writeAttribute(const std::string);
	void writeString(const std::string);
	void writeStartElementNewTag(const std::string);
	void writeCloseNewTag();
	void writeStartElementBox(const std::string);

private:
	std::ofstream outFile;
	int indent;
	int openTags;
	int openElements;
	std::vector<std::string> tempOpenTag;
	std::vector<std::string> tempElementTag;
};

#endif#pragma once
