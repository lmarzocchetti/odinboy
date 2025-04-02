package main;

import "src"

main :: proc() {
    game_boy := src.gb_create()
    src.gb_run(&game_boy)
}