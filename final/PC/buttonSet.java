package newproject;
import java.io.*;
import java.net.Socket;
import java.util.ArrayList;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;


import javax.swing.JFrame;

import javax.swing.*;
import javax.swing.JButton;


// This class handles button creation

public class buttonSet {
    public JButton but;
    public char let;
    public String pus;
    public String rel;
    
    public buttonSet(String l, char s, String p, String r){
        but = new JButton(l);
        let = s;
        pus = p;
        rel = r;
    }
    
    public JButton getButton() {
        return but;
    }
    
    public char getLetter() {
        return let;
    }
    
    public String getPush() {
        return pus;
    }
    
    public String getRelease() {
        return rel;
    }
}
