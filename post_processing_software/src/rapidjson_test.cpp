// rapidjson/example/simpledom/simpledom.cpp`
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "rapidjson/filereadstream.h"

#include <iostream>
using namespace rapidjson;
int main(int argc, char *argv[]) {
    // 1. Parse a JSON string into DOM.
    const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    Document d;
    d.Parse(json);
    // 2. Modify it by DOM.
    Value& s = d["stars"];
    s.SetInt(s.GetInt() + 1);
    // 3. Stringify the DOM
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    d.Accept(writer);
    // Output {"project":"rapidjson","stars":11}
    std::cout << buffer.GetString() << std::endl;

    if (argc < 2) return 0;

    FILE* fp = fopen(argv[1], "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document e;
    e.ParseStream(is);
    fclose(fp);

    static const char* kTypeNames[] =
        { "Null", "False", "True", "Object", "Array", "String", "Number" };

    for(auto &m: e.GetObject()){
        printf("Type of menber %s is %s\n",m.name.GetString(), kTypeNames[m.value.GetType()]);
    }

    const Value& angular_velocity_rad_per_sec = e["angular_velocity_rad_per_sec"];
    for(auto& v: angular_velocity_rad_per_sec.GetArray()){
        printf("{\n");
        for(auto &z: v.GetArray()){
            printf("%f\n",z.GetDouble());
        }
        printf("}\n");
    }

    return 0;
}
