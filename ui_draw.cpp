#include "ui_draw.h"
#include <ctime>
#include <cstdio>

void draw_overlay(cv::Mat& frame, State s, int fps, bool emergency,
                   int warning_seconds, double stable_seconds, int photo_count) {
    int right_x = frame.cols - 180;
    char fps_str[32];
    snprintf(fps_str, sizeof(fps_str), "FPS: %d", fps);
    cv::putText(frame, fps_str, cv::Point(right_x, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    if (s != IDLE && !emergency) {
        char timer_str[64];
        snprintf(timer_str, sizeof(timer_str), "Timer: %.1f s", stable_seconds);
        cv::putText(frame, timer_str, cv::Point(right_x, 60), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
    }

    if (s == WARNING && !emergency) {
        char countdown_str[32];
        snprintf(countdown_str, sizeof(countdown_str), "Alert: %d s", warning_seconds);
        cv::putText(frame, countdown_str, cv::Point(right_x, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }

    time_t now = time(nullptr);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    cv::putText(frame, time_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    const char* state_text = "";
    cv::Scalar state_color;
    if (emergency) {
        state_text = "FIRE ALERT!";
        state_color = cv::Scalar(0, 0, 255);
    } else {
        switch (s) {
            case IDLE:      state_text = "FREE"; state_color = cv::Scalar(0, 255, 0); break;
            case WARNING:   state_text = "WARNING"; state_color = cv::Scalar(0, 255, 255); break;
            case OCCUPIED:  state_text = "OCCUPIED"; state_color = cv::Scalar(0, 0, 255); break;
        }
    }
    cv::putText(frame, state_text, cv::Point(10, 65), cv::FONT_HERSHEY_SIMPLEX, 1.0, state_color, 2);

    int y3 = 100;
    if (emergency) {
        cv::putText(frame, "FLAME DETECTED! BUZZER ON", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        y3 += 30;
        cv::putText(frame, "PLEASE TAKE ACTION", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    } else if (s == WARNING) {
        cv::putText(frame, "Object detected, stable?", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        y3 += 30;
        cv::putText(frame, "BUZZER: OFF", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    } else if (s == OCCUPIED) {
        cv::putText(frame, "OCCUPIED! BUZZER INTERMITTENT", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        y3 += 30;
        cv::putText(frame, "PLEASE CLEAR THE AREA", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    } else {
        if (!bg_ready) {
            cv::putText(frame, "Press 's' to save background", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            y3 += 30;
        } else if (!detection_started) {
            cv::putText(frame, "Press 'c' to start detection", cv::Point(10, y3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            y3 += 30;
        }
    }

    char photo_str[64];
    snprintf(photo_str, sizeof(photo_str), "Photos: %d (Press 'p' to save)", photo_count);
    cv::putText(frame, photo_str, cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
}