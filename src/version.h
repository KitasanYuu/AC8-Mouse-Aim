#pragma once
// The mod's version, in one place: the upstream release this fork is based on, then this fork's
// own release number after "+Alf." (SemVer build metadata), e.g. 0.2.30+Alf.1.0.0. A release
// raises the own number; taking in a newer upstream release changes the base.
#define MOUSEFLIGHT_UPSTREAM "0.2.30"
#define MOUSEFLIGHT_RELEASE "1.0.0"
#define MOUSEFLIGHT_VERSION MOUSEFLIGHT_UPSTREAM "+Alf." MOUSEFLIGHT_RELEASE
#define MOUSEFLIGHT_VERSION_W L"" MOUSEFLIGHT_VERSION
