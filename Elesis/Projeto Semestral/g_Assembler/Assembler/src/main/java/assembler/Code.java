/**
 * Curso: Elementos de Sistemas
 * Arquivo: Code.java
 */

package assembler;

import java.util.*;


public class Code {

    public static String dest(String[] mnemonic) {
        String op = mnemonic[0];
        if (op.equals("jmp") || op.equals("je") || op.equals("jne") ||
                op.equals("jg") || op.equals("jge") || op.equals("jl") ||
                op.equals("jle") || op.equals("nop")) {
            return "0000";
        }
        int d3 = 0; // n usado
        int d2 = 0; // (%A)
        int d1 = 0; // %D
        int d0 = 0; // %A

        List<String> destOperands = new ArrayList<>();

        if (op.equals("movw")) {
            //
            for (int i = 2; i < mnemonic.length; i++) {
                destOperands.add(mnemonic[i]);
            }
        } else if (op.equals("addw") || op.equals("subw") || op.equals("rsubw") ||
                op.equals("andw") || op.equals("orw")) {
            //
            if (mnemonic.length > 3) {
                destOperands.add(mnemonic[3]);
            }
        } else if (op.equals("incw") || op.equals("decw") ||
                op.equals("notw") || op.equals("negw")) {
            //
            if (mnemonic.length > 1) {
                destOperands.add(mnemonic[1]);
            }
        }

        for (String dest : destOperands) {
            if (dest.equals("%A")) {
                d0 = 1;
            } else if (dest.equals("%D")) {
                d1 = 1;
            } else if (dest.equals("(%A)")) {
                d2 = 1;
            }
        }

        return "" + d3 + d2 + d1 + d0;
    }

    public static String comp(String[] mnemonic) {
        String op = mnemonic[0];
        String key = "";
        String compCode = "";

        Map<String, String> compTable = new HashMap<>();

        //
        compTable.put("movw_%A", "000110000");
        compTable.put("movw_%D", "000001100");
        compTable.put("movw_(%A)", "001110000");
        compTable.put("addw_%A_%D", "000000010");
        compTable.put("addw_(%A)_%D", "001000010");
        compTable.put("addw_$1_(%A)", "001110111");
        compTable.put("incw_%A", "000110111");
        compTable.put("incw_%D", "000011111");
        compTable.put("incw_(%A)", "001110111");
        compTable.put("decw_%A", "000110010");
        compTable.put("decw_%D", "000001110");
        compTable.put("decw_(%A)", "001110010");
        compTable.put("notw_%A", "000110001");
        compTable.put("notw_%D", "000001101");
        compTable.put("negw_%A", "000110011");
        compTable.put("negw_%D", "000001111");
        compTable.put("andw_%D_%A", "000000000");
        compTable.put("andw_(%A)_%D", "001000000");
        compTable.put("orw_%D_%A", "000010101");
        compTable.put("orw_(%A)_%D", "001010101");
        compTable.put("subw_%D_(%A)", "001010011");
        compTable.put("rsubw_%D_(%A)", "001000111");
        compTable.put("jmp", "000001100");
        compTable.put("je", "000001100");
        compTable.put("jne", "000001100");
        compTable.put("jg", "000001100");
        compTable.put("jge", "000001100");
        compTable.put("jl", "000001100");
        compTable.put("jle", "000001100");

        //
        if (op.equals("movw")) {
            //
            if (mnemonic.length > 1) {
                key = "movw_" + mnemonic[1];
            }
        } else if (op.equals("addw") || op.equals("subw") || op.equals("rsubw") ||
                op.equals("andw") || op.equals("orw")) {
            //
            if (mnemonic.length > 2) {
                key = op + "_" + mnemonic[1] + "_" + mnemonic[2];
            }
        } else if (op.equals("incw") || op.equals("decw") ||
                op.equals("notw") || op.equals("negw")) {
            //
            if (mnemonic.length > 1) {
                key = op + "_" + mnemonic[1];
            }
        } else if (op.equals("jmp") || op.equals("je") || op.equals("jne") ||
                op.equals("jg") || op.equals("jge") || op.equals("jl") ||
                op.equals("jle")) {
            key = op;
        }

        compCode = compTable.get(key);

        if (compCode != null) {
            return compCode;
        } else {
            //
            if (op.equals("addw") && mnemonic[1].equals("$1")) {
                if (mnemonic[2].equals("(%A)")) {
                    return "001110111"; // (%A)+1
                } else if (mnemonic[2].equals("%A")) {
                    return "000110111"; // %A+1
                } else if (mnemonic[2].equals("%D")) {
                    return "000011111"; // %D+1
                }
            } else if (op.equals("subw") && mnemonic[2].equals("$1")) {
                if (mnemonic[1].equals("(%A)")) {
                    return "001110010"; // (%A)-1
                } else if (mnemonic[1].equals("%A")) {
                    return "000110010"; // %A-1
                } else if (mnemonic[1].equals("%D")) {
                    return "000001110"; // %D-1
                }
            }
        }

        return ""; //excessao
    }


    public static String jump(String[] mnemonic) {
        String op = mnemonic[0];
        switch(op) {
            case "jmp": return "111";
            case "je": return "010";
            case "jne": return "101";
            case "jg": return "001";
            case "jge": return "011";
            case "jl": return "100";
            case "jle": return "110";
            default: return "000";
        }
    }


    public static String toBinary(String symbol) {
        int num = Integer.parseInt(symbol);
        if (num < 0 || num > 65535) {
            throw new IllegalArgumentException("Number out of range");
        }
        String binaryString = Integer.toBinaryString(num);
        return String.format("%16s", binaryString).replace(' ', '0');
    }

}
