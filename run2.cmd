.\bin\yraytrace --scene tests\06_metal\metal.json --output out\lowres\06_metal_720_256.jpg --samples 256 --resolution 720
.\bin\yraytrace --scene tests\07_plastic\plastic.json --output out\lowres\07_plastic_720_256.jpg --samples 256 --resolution 720
.\bin\yraytrace --scene tests\08_glass\glass.json --output out\lowres\08_glass_720_256.jpg --samples 256 --bounces  8 --resolution 720
.\bin\yraytrace --scene tests\09_opacity\opacity.json --output out\lowres\09_opacity_720_256.jpg --samples 256 --resolution 720
.\bin\yraytrace --scene tests\10_hair\hair.json --output out\lowres\10_hair_720_256.jpg --samples 256 --resolution 720
.\bin\yraytrace --scene tests\11_bathroom1\bathroom1.json --output out\lowres\11_bathroom1_720_256.jpg --samples 256 --bounces  8 --resolution 720
.\bin\yraytrace --scene tests\12_ecosys\ecosys.json --output out\lowres\12_ecosys_720_256.jpg --samples 256 --resolution 720


.\bin\yraytrace --scene tests\refract\refract.json --output out\lowres\refract.jpg --samples 256 --bounces  8 --resolution 720


.\bin\yraytrace --scene tests\13_cornellboxvolumetric\cornellbox.json --output out\lowres\13_cornellbox_512_256.jpg --samples 256 --resolution 512



.\bin\yraytrace --scene tests\15_matcap\matcap.json --output out\extra\0x_matcap_720_9.jpg --samples 9 --shader matcap --resolution 720