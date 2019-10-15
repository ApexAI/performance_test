// Copyright 2019 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERFORMANCE_TEST__VERSION_H_
#define PERFORMANCE_TEST__VERSION_H_

#ifdef PERFORMANCE_TEST_VERSION
const char * version = PERFORMANCE_TEST_VERSION;
#else
const char * version = "unknown";
#endif

#endif  // PERFORMANCE_TEST__VERSION_H_
