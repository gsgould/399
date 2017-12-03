package newproject;

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
