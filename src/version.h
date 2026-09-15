#pragma once

// Single source of the firmware version. A release is a git tag "v" + this
// string; the release workflow refuses to publish when the two disagree, so
// bump this in the same commit you intend to tag.
#define FW_VERSION "1.1.0"
