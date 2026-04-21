#!/usr/bin/env python3

from manager import DroneManager
from interface import DroneVisualizer


def main():
    manager = DroneManager()
    app = DroneVisualizer(manager)
    
    try:
        app.generate_drones()      
        app.run()
        
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        try:
            app.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()