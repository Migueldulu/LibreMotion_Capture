using UnrealBuildTool;
using System.IO;

public class Telemetry : ModuleRules
{
    public Telemetry(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            "HeadMountedDisplay",
            "XRBase", // en versiones nuevas solo encuentra HeadMountedDisplay aqui
            "InputCore",
            "EnhancedInput"
        });

        PrivateDependencyModuleNames.AddRange(new string[] { 
            "HeadMountedDisplay",
            "XRBase"
        });

        if (Target.Platform == UnrealTargetPlatform.Android)
        {
            // Necesario para Android/AndroidApplication.h (GetJavaEnv, GetGameActivityThis, etc)
            PrivateDependencyModuleNames.Add("Launch");
            // 1) AAR (Java + permiso INTERNET + proguard keep), via APL
            string APL = System.IO.Path.Combine(ModuleDirectory, "Telemetry_APL.xml");
            AdditionalPropertiesForReceipt.Add("AndroidPlugin", APL);
            // 2) .so “link-only”: el AAR pone el .so dentro del APK,pero el enlazador necesita verlo en build
            string LibForLink = System.IO.Path.GetFullPath(System.IO.Path.Combine(ModuleDirectory, "../ThirdParty/Android/arm64-v8a/libtelemetria.so"));
            if (System.IO.File.Exists(LibForLink))
            {
                PublicAdditionalLibraries.Add(LibForLink);
            }
            else
            {
                System.Console.WriteLine("WARNING: libtelemetria.so not found for link-only: " + LibForLink);
            }
        }
    }
}
