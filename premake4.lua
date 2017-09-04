--
-- premake4 file to build DeepLoco
-- Copyright (c) 2009-2015 Glen Berseth
-- See license.txt for complete license.
--

local action = _ACTION or ""
local todir = "./" .. action

function file_exists(name)
   local f=io.open(name,"r")
   if f~=nil then io.close(f) return true else return false end
end

local linuxLibraryLoc = "./external/"
local windowsLibraryLoc = "../library/"

solution "DeepLoco"
	configurations { 
		"Debug",
		"Release"
	}
	
	platforms {
		"x32", 
		"x64"
	}
	location (todir)

	-- extra warnings, no exceptions or rtti
	flags { 
		-- "ExtraWarnings",
--		"FloatFast",
--		"NoExceptions",
--		"NoRTTI",
		"Symbols"
	}
	-- defines { "ENABLE_GUI", "ENABLE_GLFW" }

	-- debug configs
	configuration { "Debug*"}
		defines { "DEBUG" }
		flags {
			"Symbols",
			Optimize = Off
		}
		targetdir ( "./x64/Debug" )
 
 	-- release configs
	configuration {"Release*"}
		defines { "NDEBUG" }
		flags { "Optimize" }
		targetdir ( "./x64/Release" )

	-- windows specific
	configuration {"windows"}
		defines { "WIN32", "_WINDOWS" }
		libdirs { "lib" }
		targetdir ( "./x64/Debug" )
		flags {"StaticRuntime"}

	configuration {"windows", "Debug*"}
		targetdir ( "./x64/Debug" )
	configuration {"windows", "Release*"}
		targetdir ( "./x64/Release" )

	configuration { "linux", "Debug*", "gmake"}
		linkoptions { 
			-- "-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		targetdir ( "./x64/Debug" )

	configuration { "linux", "Release*", "gmake"}
		linkoptions { 
			-- "-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		targetdir ( "./x64/Release" )

	configuration { "macosx" }
        buildoptions { "-stdlib=libc++ -std=c++11 -ggdb" }
		linkoptions { 
			"-stdlib=libc++" ,
			"-Wl,-rpath," .. path.getabsolute("lib")
		}
		links {
	        "OpenGL.framework",
        }
        targetdir ( "./bin/Debug/" )
      
	if os.get() == "macosx" then
		premake.gcc.cc = "clang"
		premake.gcc.cxx = "clang++"
		-- buildoptions("-std=c++0x -ggdb -stdlib=libc++" )
	end


project "DeepLoco"
	language "C++"
	kind "ConsoleApp"

	files { 
		-- Source files for this project
		-- "render/*.cpp",
		-- "util/*.cpp",
		-- "sim/*.cpp",
		-- "anim/*.cpp",
		-- "learning/*.cpp",
		-- "scenarios/*.cpp",
		"external/LodePNG/*.cpp",
		"Main.cpp"
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"anim",
		"learning",
		"sim",
		"render",
		"scenarios",
		"util"
	}
	links {
		"deepLocoScenarios",
		"deepLocoLearning",
		"deepLocoAnim",
		"deepLocoSim",
		"deepLocoUtil",
		"deepLocoRender",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
				"GLU",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
				"GLU",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}

project "deepLocoUtil"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"util/*.cpp",
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"util"
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


project "deepLocoLearning"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"learning/*.cpp",
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"learning",
		"util",
	}
	links {
		"deepLocoUtil",
		"deepLocoAnim",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


project "deepLocoAnim"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"anim/*.cpp",
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"anim"
	}
	links {
		"deepLocoUtil",
		-- "deepLocoSim",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


project "deepLocoSim"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"sim/*.cpp",
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"sim",
		-- "util"
	}
	links {
		"deepLocoUtil",
		"deepLocoAnim",
		"deepLocoLearning",
		-- "deepLocoRender",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}


project "deepLocoRender"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"render/*.cpp",
		"render/*.h",
		"../library/LodePNG/*.cpp",
		"../library/LodePNG/*.h",
	}
	excludes 
	{
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
		"render/OFFParser.*",
	}	
	includedirs { 
		"./",
	}
	links {
		"deepLocoUtil",
		"deepLocoAnim"
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
				"GLU",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
				"GLU",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}

project "deepLocoScenarios"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"scenarios/*.cpp",
	}
	excludes 
	{
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"scenarios"
	}
	links {
		"deepLocoLearning",
		"deepLocoAnim",
		"deepLocoSim",
		"deepLocoUtil",
		"deepLocoRender",
	}

	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}
			-- debug configs
		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			defines { "NDEBUG" }
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"glut",
				"GLEW",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"dl",
			"pthread"
		}
		


if os.get() == "windows" then
   project "caffe"
	language "C++"
	kind "StaticLib"

	files { 
		-- Source files for this project
		--"util/*.h",
		"../library/caffe/src/caffe/*.cpp",
		"../library/caffe/src/caffe/*.hpp",
		"../library/caffe/src/caffe/layers/*.cpp",
		"../library/caffe/src/caffe/layers/*.hpp",
		"../library/caffe/src/caffe/proto/*.cc",
		"../library/caffe/src/caffe/proto/*.h",
		"../library/caffe/src/caffe/solvers/*.cpp",
		"../library/caffe/src/caffe/solvers/*.hpp",
		"../library/caffe/src/caffe/util/*.cpp",
		"../library/caffe/src/caffe/util/*.hpp",
	}
	excludes 
	{
		"../library/caffe/src/caffe/util/signal_handler.cpp",
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
	}	
	includedirs { 
		"./",
		"../library/caffe/src/caffe"
	}
	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb" )	

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}
		
end

--
-- premake4 file to build DeepLoco_Optimizer
-- Copyright (c) 2009-2015 Glen Berseth
-- See license.txt for complete license.
--

-- local linuxLibraryLoc = "../external/"
-- local windowsLibraryLoc = "../../library/"

project "DeepLoco_Optimizer"
	language "C++"
	kind "ConsoleApp"

	files { 
		-- Source files for this project
		-- "../learning/*.cpp",
		-- "../scenarios/*.cpp",
		-- "../sim/*.cpp",
		-- "../util/*.cpp",
		-- "../anim/*.cpp",
		"optimizer/opt/*.cpp",
		"optimizer/opt/*.c",
		"optimizer/opt/*.h",
		"optimizer/scenarios/*.cpp",
		"optimizer/Main.cpp",
		

	}
	excludes {
		"scenarios/Draw*.h",
		"scenarios/Draw*.cpp",
		"sim/CharTracer.cpp",
		"learning/DMACETrainer - Copy.cpp",
		"scenarios/ScenarioExpImitate - Copy.cpp",
		"**- Copy**.cpp",
		"learning/CaclaTrainer - Copy \(2\).cpp",
		"learning/CaclaTrainer - Copy.cpp",
		"optimizer/opt/QL.c",
		"optimizer/opt/QuadProg.cpp",
	}

	includedirs { 
		"optimizer",
		"./",
		"optimizer/opt",
		"anim",
		"learning",
		"sim",
		"render",
		"scenarios",
		"util"
	}
	links {
		"deepLocoScenarios",
		"deepLocoLearning",
		"deepLocoAnim",
		"deepLocoSim",
		"deepLocoUtil",
		"deepLocoRender",
	}
	

	defines {
		"_CRT_SECURE_NO_WARNINGS",
		"_SCL_SECURE_NO_WARNINGS",
		"CPU_ONLY",
		"GOOGLE_GLOG_DLL_DECL=",
		"ENABLE_TRAINING",
	}

	-- targetdir "./"
	buildoptions("-std=c++0x -ggdb -g" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }

		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		libdirs { 
			-- "lib",
			linuxLibraryLoc .. "Bullet/bin",
			linuxLibraryLoc .. "jsoncpp/build/debug/src/lib_json",
			linuxLibraryLoc .. "caffe/build/lib",
		}
		
		includedirs { 
			linuxLibraryLoc .. "Bullet/src",
			linuxLibraryLoc,
			linuxLibraryLoc .. "jsoncpp/include",
			linuxLibraryLoc .. "caffe/include/",
			linuxLibraryLoc .. "caffe/build/src/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			linuxLibraryLoc .. "3rdparty/include/hdf5",
			linuxLibraryLoc .. "3rdparty/include/",
			linuxLibraryLoc .. "3rdparty/include/openblas",
			linuxLibraryLoc .. "3rdparty/include/lmdb",
			"/usr/local/cuda/include/",
			linuxLibraryLoc .. "OpenCV/include",
			linuxLibraryLoc .. "caffe/src/",
			linuxLibraryLoc .. "CMA-ESpp/cma-es",
			"/usr/include/hdf5/serial/",
		}
		defines {
			"_LINUX_",
		}

		configuration { "linux", "Debug*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_debug",
				"BulletCollision_gmake_x64_debug",
				"LinearMath_gmake_x64_debug",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"f2c",
			}
	 
	 	-- release configs
		configuration { "linux", "Release*", "gmake"}
			links {
				"X11",
				"dl",
				"pthread",
				-- Just a few dependancies....
				"BulletDynamics_gmake_x64_release",
				"BulletCollision_gmake_x64_release",
				"LinearMath_gmake_x64_release",
				"jsoncpp",
				"boost_system",
				"caffe",
				"glog",
				--"hdf5",
				--"hdf5_hl",
				"hdf5_serial_hl",
				"hdf5_serial",
				"f2c",
			}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		linkoptions  { 
			"libopenblas.dll.a",
			-- "`pkg-config --cflags glu`" 
		}
		defines {
			"_USE_MATH_DEFINES"	
		}
		includedirs { 
			windowsLibraryLoc .. "Bullet/include",
			windowsLibraryLoc,
			windowsLibraryLoc .. "Json_cpp",
			windowsLibraryLoc .. "caffe/include/",
			"C:/Program Files (x86)/boost/boost_1_58_0/",
			windowsLibraryLoc .. "caffe/3rdparty/include/hdf5",
			windowsLibraryLoc .. "caffe/3rdparty/include/",
			windowsLibraryLoc .. "caffe/3rdparty/include/openblas",
			windowsLibraryLoc .. "caffe/3rdparty/include/lmdb",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/include/",
			windowsLibraryLoc .. "OpenCV/include",
			windowsLibraryLoc .. "caffe/src/",
		}	
		
		libdirs { 
			windowsLibraryLoc .. "lib",
			windowsLibraryLoc .. "boost_lib",
			windowsLibraryLoc .. "Bullet/Debug/x64",
			windowsLibraryLoc .. "Json_cpp/x64",
			"C:/Program Files (x86)/boost/boost_1_58_0/stage/lib",
			"C:/Program Files (x86)/boost/boost_1_58_0/libs",
			windowsLibraryLoc .. "caffe/3rdparty/lib",
			windowsLibraryLoc .. "OpenCV/x64/vc12/staticlib",
			"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.5/lib/x64",
		}
		
		-- release configs
		configuration { "windows", "Debug*"}
			defines { "DEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics_Debug",
				"BulletCollision_Debug",
				"LinearMath_Debug",
				"jsoncpp_Debug",
				"opencv_core300d",
				"opencv_calib3d300d",
				"opencv_flann300d",
				"opencv_highgui300d",
				"opencv_imgproc300d",
				"opencv_imgcodecs300d",
				"opencv_ml300d",
				"opencv_objdetect300d",
				"opencv_photo300d",
				"opencv_features2d300d",
				"opencv_stitching300d",
				"opencv_video300d",
				"opencv_videostab300d",
				"opencv_hal300d",
				"libjpegd",
				"libjasperd",
				"libpngd",
				"IlmImfd",
				"libtiffd",
				"libwebpd",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflagsd",
				"libglogd",
				"libprotobufd",
				"libprotocd",
				"leveldbd",
				"lmdbd",
				"libhdf5_D",
				"libhdf5_hl_D",
				"Shlwapi",
				"zlibd",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

		-- release configs
		configuration { "windows", "Release*"}
			defines { "NDEBUG" }
			links { 
				"opengl32",
				"glu32",
				-- Just a few dependancies....
				"BulletDynamics",
				"BulletCollision",
				"LinearMath",
				"jsoncpp",
				"opencv_core300",
				"opencv_calib3d300",
				"opencv_flann300",
				"opencv_highgui300",
				"opencv_imgproc300",
				"opencv_imgcodecs300",
				"opencv_ml300",
				"opencv_objdetect300",
				"opencv_photo300",
				"opencv_features2d300",
				"opencv_stitching300",
				"opencv_video300",
				"opencv_videostab300",
				"opencv_hal300",
				"libjpeg",
				"libjasper",
				"libpng",
				"IlmImf",
				"libtiff",
				"libwebp",
				-- "cudart",
				-- "cuda",
				-- "nppi",
				-- "cufft",
				-- "cublas",
				-- "curand",
				"gflags",
				"libglog",
				"libprotobuf",
				"libprotoc",
				"leveldb",
				"lmdb",
				"libhdf5",
				"libhdf5_hl",
				"Shlwapi",
				"zlib",
				-- "libopenblas"
				-- "libopenblas.dll.a",
				"glew32",
				"caffe",
			}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"Cocoa.framework",
			"dl",
			"pthread"
		}




