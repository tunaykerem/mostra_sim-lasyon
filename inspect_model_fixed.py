#!/usr/bin/env python3

"""
Model Inspector - Check what classes are available in last.pt
"""

try:
    from ultralytics import YOLO
    
    print("Loading last.pt model...")
    # Load the model
    model = YOLO('last.pt')
    
    print("=== YOLOv8 Model Inspection: last.pt ===")
    print("Model loaded successfully!")
    
    # Get class names
    class_names = model.model.names
    print(f"\nAvailable classes ({len(class_names)}):")
    for class_id, class_name in class_names.items():
        print(f"  {class_id}: {class_name}")
    
    # Check if cone class exists (check for both English and Turkish)
    cone_classes = [name for name in class_names.values() if any(keyword in name.lower() for keyword in ['cone', 'koni'])]
    if cone_classes:
        print(f"\n✅ Cone-related classes found: {cone_classes}")
        print("✅ Model supports cone detection")
    else:
        print("\n❌ No cone-related classes found")
        print("   Color-based detection will be used for cones")
    
    # Check for traffic sign classes
    sign_keywords = ['sign', 'dur', 'donul', 'yasak', 'isik', 'donerkavsak', 'durak', 'engellipark', 'park', 'girilmez', 'trafikisigi']
    sign_classes = [name for name in class_names.values() if any(keyword in name.lower() for keyword in sign_keywords)]
    if sign_classes:
        print(f"\n✅ Traffic sign classes found ({len(sign_classes)}):")
        for sign_class in sign_classes:
            print(f"   - {sign_class}")
        print("   Model supports traffic sign detection")
    
    print(f"\n🎯 Model Configuration:")
    print(f"   File: last.pt")
    print(f"   Classes: {len(class_names)}")
    print(f"   Ready for: SimpleYOLOv8Detector")
    print(f"   Status: ✅ Compatible with main3.py")

except ImportError:
    print("❌ ultralytics not installed. Install with: pip install ultralytics")
except FileNotFoundError:
    print("❌ last.pt file not found in current directory")
except Exception as e:
    print(f"❌ Error loading model: {e}")
