	
	hasCL = findOpenCL_NVIDIA()
	
	if (hasCL) then

		project "OpenCL_GL_interop_NVIDIA"

		initOpenCL_NVIDIA()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

		initOpenGL()
		initGlut()
		initGlew()

	
		includedirs {
			"../../../rendering/BulletMath"
		}
		
		files {
			"../main.cpp",
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h",
			"../btOpenCLGLInteropBuffer.cpp",
			"../btOpenCLGLInteropBuffer.h",
			"../btStopwatch.cpp",
			"../btStopwatch.h"
		}
		
	end