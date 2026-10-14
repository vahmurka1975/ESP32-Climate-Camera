#pragma once
// Для личных настроек скопируйте Secrets.example.h в Secrets.h.
// Secrets.h исключён из Git. Пример позволяет собрать проект без личных данных.
#if __has_include("Secrets.h")
#include "Secrets.h"
#else
#include "Secrets.example.h"
#endif
#define CC_HOSTNAME "climate-camera"
#define CC_HTTP_PORT 80
