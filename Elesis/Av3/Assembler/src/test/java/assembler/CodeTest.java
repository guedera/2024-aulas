/**
 * Curso: Elementos de Sistemas
 * Arquivo: CodeTest.java
 * Created by Luciano Soares <lpsoares@insper.edu.br>
 * Date: 16/04/2017
 */

package assembler;
import org.junit.Test;
import org.junit.AfterClass;
import org.junit.Rule;

import java.io.IOException;
import java.io.PrintWriter;

import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class CodeTest  {

	private static StringBuilder builder = new StringBuilder();

	/**
	 * Create the test case
	 */
	public CodeTest() {
	}

	
	/**
	 * Teste para geração de código para Destino
	 */
	@Test
	public void testCode_Computation() {

		try {
			assertEquals("000110000", Code.comp(new String[] { "movw","%A","%D" }));
			assertEquals("000110000", Code.comp(new String[] { "movw","%A","(%D)" }));
			assertEquals("000001100", Code.comp(new String[] { "movw","%D","%A" }));
			assertEquals("000001100", Code.comp(new String[] { "movw","%D","(%D)" }));
			assertEquals("001110000", Code.comp(new String[] { "movw","(%A)","%D" }));
			assertEquals("001110000", Code.comp(new String[] { "movw","(%A)","(%D)" }));
			assertEquals("010001100", Code.comp(new String[] { "movw","(%D)","%A" }));
			assertEquals("010001100", Code.comp(new String[] { "movw","(%D)","(%D)" }));
			assertEquals("000101010", Code.comp(new String[] { "movw","$0","%A" }));
			assertEquals("000101010", Code.comp(new String[] { "movw","$0","(%D)" }));
			assertEquals("000111111", Code.comp(new String[] { "movw","$1","%A" }));
			assertEquals("000111111", Code.comp(new String[] { "movw","$1","(%D)" }));
			assertEquals("000111010", Code.comp(new String[] { "movw","$-1","%A" }));
			assertEquals("000111010", Code.comp(new String[] { "movw","$-1","(%D)" }));
		} catch(Exception e) {
			e.printStackTrace();
		}

	}
	
	/**
	 * Teste para geração de código para Destino
	 */
	@Test
	public void testCode_Destine() {

		try {
			org.junit.Assume.assumeNotNull( Code.dest(new String[] {"nop"}) );		// ignora test
		} catch(Exception e) {
			org.junit.Assume.assumeNoException(e);
		}
		try {

			assertEquals("0001", Code.dest(new String[] { "movw","%D","%A" }));
			assertEquals("0010", Code.dest(new String[] { "movw","%A","%D" }));
			assertEquals("1000", Code.dest(new String[] { "movw","%A","(%D)" }));
			assertEquals("0100", Code.dest(new String[] { "movw","%A","(%A)" }));
			assertEquals("0011", Code.dest(new String[] { "movw","(%A)","%D","%A" }));
			assertEquals("0011", Code.dest(new String[] { "movw","(%A)","%A","%D" }));
			assertEquals("0110", Code.dest(new String[] { "movw","%A","%D","(%A)" }));
			assertEquals("1001", Code.dest(new String[] { "movw","%D","%A","(%D)" }));
			assertEquals("0110", Code.dest(new String[] { "movw","%A","(%A)","%D" }));
			assertEquals("1001", Code.dest(new String[] { "movw","%A","(%D)","%A" }));
			assertEquals("1101", Code.dest(new String[] { "movw","%A","(%D)","%A","(%A)" }));
			assertEquals("0111", Code.dest(new String[] { "movw","%A","%D","%A","(%A)" }));
			assertEquals("1110", Code.dest(new String[] { "movw","%A","(%D)","%D","(%A)" }));
			assertEquals("1111", Code.dest(new String[] { "movw","%A","%D","%A","(%A)","(%D)" }));

		} catch(Exception e) {
			e.printStackTrace();
		}

	}


}
