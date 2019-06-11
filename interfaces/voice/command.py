import speech_recognition as sr
import argparse


def main(args):
    r = sr.Recognizer()
    r.recognize_google()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Voice Command Interface')
    args = parser.parse_args()
    main(args)