package frc.robot;

import java.awt.*;
import java.awt.event.*;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import javax.swing.*;

public class LanguageInterpreter {

    private String text;
    private String javaForm;
    private List<Callables> callablesList;

    // Static reference to the info frame so we can close it later
    private static JFrame infoFrameInstance = null; // MODIFICATION

    public LanguageInterpreter(String path, boolean useFile) {
        this.callablesList = buildCallables();

        text = "";
        if (useFile) {
            File inputFile = new File(path);
            System.out.println(
                    "Attempting to read pseudo-code from file: " + inputFile.getAbsolutePath());
            StringBuilder strBuilder = new StringBuilder();
            if (inputFile.exists() && inputFile.isFile()) {
                try (Scanner scanner = new Scanner(inputFile)) {
                    while (scanner.hasNextLine()) {
                        strBuilder.append(scanner.nextLine()).append("\n");
                    }
                } catch (IOException e) {
                    System.err.println("Error reading file: " + path);
                    e.printStackTrace();
                }
            } else {
                System.err.println(
                        "Pseudo-code input file not found or is not a file: "
                                + inputFile.getAbsolutePath());
            }
            text = strBuilder.toString();
        } else {
            System.out.println("Launching GUI for pseudo-code input...");
            // The call to showInfoFrame is now inside getText() -> showEditorAndGetText()
            text = getText();
            if (text == null) {
                text = "";
                System.out.println(
                        "User closed the dialog without clicking 'Generate' or an error occurred.");
            } else {
                System.out.println("Received text from GUI.");
            }
            infoFrameInstance.dispose();
        }

        javaForm = getJavaForm();

        try (PrintWriter pw = new PrintWriter("JavaOutput.txt")) {
            if (javaForm != null && !javaForm.isEmpty()) {
                if (javaForm.endsWith(", ")) {
                    javaForm = javaForm.substring(0, javaForm.length() - 2);
                }
                if (javaForm.length() > 1) {
                    javaForm = "sequence(" + javaForm + ")";
                }
                javaForm = javaForm.replaceAll("\n", "");
                javaForm = "driverController.a().onTrue(" + javaForm + ");";
                pw.print(javaForm);
                System.out.println("Successfully wrote processed form to JavaOutput.txt");
            } else if (javaForm == null) {
                System.err.println(
                        "Skipping output file write due to processing errors in getJavaForm().");
                pw.print("");
            } else {
                System.out.println(
                        "No text processed or empty result, writing empty JavaOutput.txt");
                pw.print("");
            }
        } catch (IOException e) {
            System.err.println("Error writing to JavaOutput.txt");
            e.printStackTrace();
        }
        System.out.println(
                "Final processed form: "
                        + (javaForm != null ? javaForm : "<Error During Processing>"));
    }

    public String getJavaForm() {
        StringBuilder java = new StringBuilder();
        if (text == null || text.trim().isEmpty()) {
            return "";
        }
        String[] lines = text.split("\n");
        for (int i = 0; i < lines.length; i++) {
            if (lines[i].trim().isEmpty()) {
                continue;
            }
            Line currentLine = new Line(lines[i], this.callablesList);
            String lineJava = currentLine.getJava();
            if ("error".equals(lineJava)) {
                System.err.println(
                        "Halting further processing due to error on line " + (i + 1) + ".");
                return null;
            } else {
                java.append(lineJava).append(", ");
            }
        }
        String result = java.toString();
        if (result.endsWith(", ")) {
            result = result.substring(0, result.length() - 2);
        }
        return result;
    }

    private List<Callables> buildCallables() {
        List<Callables> loadedCallables = new ArrayList<>();
        String configFilePath = "ConfigFile.txt";
        File configFile = new File(configFilePath);
        System.out.println(
                "Attempting to load callables from file: " + configFile.getAbsolutePath());
        if (!configFile.exists() || !configFile.isFile()) {
            System.err.println(
                    "FATAL ERROR: ConfigFile.txt not found at: "
                            + configFile.getAbsolutePath()
                            + ". Please ensure it exists at the project root.");
            return loadedCallables;
        }
        try (Scanner scanner = new Scanner(configFile)) {
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                if (!line.trim().isEmpty()) {
                    loadedCallables.add(new Callables(line));
                }
            }
        } catch (IOException e) {
            System.err.println("FATAL ERROR reading ConfigFile.txt: " + e.getMessage());
            e.printStackTrace();
        }
        System.out.println("Loaded " + loadedCallables.size() + " callables.");
        return loadedCallables;
    }

    // *** NEW METHOD to show the informational JFrame ***
    public static void showInfoFrame(String message, Component parentComponent) {
        // If an old info frame exists, dispose of it first
        if (infoFrameInstance != null && infoFrameInstance.isVisible()) {
            infoFrameInstance.dispose();
        }

        infoFrameInstance = new JFrame("Possible Commands");
        infoFrameInstance.setDefaultCloseOperation(
                JFrame.DISPOSE_ON_CLOSE); // Dispose this frame only
        infoFrameInstance.setSize(450, 1000);

        JTextArea infoTextArea = new JTextArea(message);
        infoTextArea.setWrapStyleWord(true);
        infoTextArea.setLineWrap(true);
        infoTextArea.setOpaque(false);
        infoTextArea.setEditable(false);
        infoTextArea.setFocusable(false);
        infoTextArea.setFont(UIManager.getFont("Label.font")); // Use a standard label font
        infoTextArea.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10)); // Add some padding

        infoFrameInstance.add(
                new JScrollPane(infoTextArea)); // Add scroll pane in case message is long

        // Position it relative to the parent (editor dialog) or center of screen
        if (parentComponent != null && parentComponent.isVisible()) {
            // Position it slightly offset from the editor dialog
            Point parentLocation = parentComponent.getLocation();
            infoFrameInstance.setLocation(
                    parentLocation.x + parentComponent.getWidth() + 10, parentLocation.y);
        } else {
            infoFrameInstance.setLocationRelativeTo(null); // Center on screen
        }
        infoFrameInstance.setVisible(true);
    }

    // *** MODIFIED showEditorAndGetText to manage the info frame ***
    public static String showEditorAndGetText() {
        final String[] result = new String[1];
        final JDialog dialog = new JDialog((Frame) null, "Pseudo-Code Editor", true);
        dialog.setSize(500, 400);
        dialog.setLayout(new BorderLayout());
        // IMPORTANT: We need to dispose the info frame when this dialog closes
        dialog.setDefaultCloseOperation(JDialog.DISPOSE_ON_CLOSE);

        JTextArea textArea = new JTextArea();
        textArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
        JScrollPane scrollPane = new JScrollPane(textArea);
        dialog.add(scrollPane, BorderLayout.CENTER);

        JButton runButton = new JButton("Generate");
        runButton.addActionListener(
                e -> {
                    result[0] = textArea.getText();
                    dialog.dispose(); // This will trigger the windowClosed event
                });

        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
        buttonPanel.add(runButton);
        dialog.add(buttonPanel, BorderLayout.SOUTH);

        // Add listener for window closing event (clicking 'X' or calling dispose())
        dialog.addWindowListener(
                new WindowAdapter() {
                    @Override
                    public void windowClosing(WindowEvent e) {
                        if (dialog.isVisible()) { // If closed via 'X' before button click
                            result[0] = null;
                        }
                        // Dispose the info frame when the main dialog is closing
                        if (infoFrameInstance != null && infoFrameInstance.isVisible()) {
                            infoFrameInstance.dispose();
                            infoFrameInstance = null; // Clear the reference
                        }
                    }

                    // Show info frame when editor dialog becomes visible (after
                    // pack/setLocationRelativeTo if used)
                    @Override
                    public void windowOpened(WindowEvent e) { // Or windowActivated
                        // *** CALL to show the informational frame ***
                        String infoMessage =
                                """
                                        elevatorHeight=
                                            L1
                                            L2
                                            L3
                                            L4
                                            0
                                            Processor
                                            Net
                                            Lower_Algae
                                            Upper_Algae
                                            Lollipop
                                        intake(GamePiece)
                                            Coral - intake(Coral)
                                            Algae - intake(Algae)
                                        eject(GamePiece)
                                            Coral - eject(Coral)
                                            Algae - eject(Algae)
                                        autoAlign
                                            Coral - autoAlign(Coral)
                                            Algae - autoAlign(Algae)
                                        drive(double inches)
                                        drive(double degrees)
                                        """;
                        String moreDetailInfoMessage =
                                """
                                elevatorHeight=
                                 -  L1
                                     -  The height needed to place coral in the trough
                                 -  L2
                                     -  The height needed to place coral on the shortest pole level of
                                         the reef
                                 -  L3
                                     -  The height needed to place coral on the second tallest level
                                         of the reef
                                 -  L4
                                     -  The height needed to place coral on the tallest level of the
                                         reef(changes wrist too)
                                 -  0
                                     -  This is the zero height of the elevator or at the bottom
                                 -  Processor
                                     -  The height needed to shoot algae into the processor.
                                 -  Net
                                     -  The height to shoot algae into the net(changes the wrist
                                         accordingly too)
                                 -  Lower_Algae
                                     -  The height to remove algae place on L2 of the reef(the shorter
                                         of the two algae locations)
                                 -  Upper_Algae
                                     -  The height to remove algae place on L3 of the reef(the taller
                                         of the two algae locations)
                                 -  Lollipop
                                     -  The height to removal algae from the lollipops(algae placed
                                         onto of coral near the starting zone)
                                intake(GamePiece)
                                 -  Coral - intake(Coral)
                                     -  This starts the sequence needed to intake coral
                                 -  Algae - intake(Algae)
                                     -  This starts the sequence needed to intake algae
                                eject(GamePiece)
                                 -  Coral - eject(Coral)
                                     -  This ejects the coral
                                 -  Algae - eject(Algae)
                                     -  This ejects the algae
                                autoAlign
                                 -  Coral - autoAlign(Coral)
                                     -  This aligns to the reef in the location of the closest node of
                                         the reef
                                 -  Algae - autoAlign(Algae)
                                     -  This aligns to the reef in the location of the closest side of
                                         the reef for algae
                                drive(double inches)
                                 -  Inches - this is how far forward the robot drives forward(Ex.
                                     3.5,15, 49, 63.38562)
                                drive(double degrees)
                                 -  Degrees - this is how far forward the robot rotates clockwise
                                     (Ex.3.5,15, 49, 63.38562)
                                """;

                        showInfoFrame(moreDetailInfoMessage, dialog);
                    }
                });

        dialog.setLocationRelativeTo(null);
        // setVisible(true) should be the LAST call for the dialog before it blocks
        dialog.setVisible(true);

        return result[0];
    }

    public static String getText() {
        if (SwingUtilities.isEventDispatchThread()) {
            return showEditorAndGetText();
        } else {
            final String[] resultContainer = new String[1];
            try {
                SwingUtilities.invokeAndWait(
                        () -> {
                            resultContainer[0] = showEditorAndGetText();
                        });
                return resultContainer[0];
            } catch (InterruptedException | InvocationTargetException e) {
                Thread.currentThread().interrupt();
                System.err.println("Error displaying text editor dialog: " + e.getMessage());
                e.printStackTrace();
                return null;
            }
        }
    }

    public static void main(String[] args) {
        System.out.println("Running LanguageInterpreter directly (main method)...");
        LanguageInterpreter languageInterpreterGUI = new LanguageInterpreter(null, false);
    }
}

// Inner class Callables
class Callables {
    public String sudoForm;
    public String javaBefore;
    public String javaAfter;
    public ArrayList<String> javaOptions;
    public ArrayList<String> sudoOptions;
    public boolean typeBased = false;
    public String type = "";
    public String sudoFormAfter;

    public Callables() {
        sudoForm = "";
        javaBefore = "";
        javaAfter = "";
        javaOptions = new ArrayList<>();
        sudoOptions = new ArrayList<>();
    }

    // Constructor parsing lines from ConfigFile.txt
    public Callables(String importLine) {
        javaOptions = new ArrayList<>();
        sudoOptions = new ArrayList<>();

        if (importLine == null || importLine.trim().isEmpty()) {
            // System.err.println("WARN: Callables constructor received empty/null line.");
            return;
        }

        if (importLine.contains("#")) { // Simplest form: sudo#javaBefore
            String[] work = importLine.split("#", -1);
            this.sudoForm = (work.length > 0) ? work[0].trim() : "";
            this.javaBefore = (work.length > 1) ? work[1].trim() : "";
            this.javaAfter = ""; // Default for this format
            // Add default empty options for consistency, though likely not used by this format's
            // logic
            javaOptions.add("");
            sudoOptions.add("");
        } else if (importLine.contains(
                "%")) { // Type-based: sudo%type%sudoAfter%javaBefore%javaAfter
            typeBased = true;
            String[] work = importLine.split("%", -1);
            if (work.length >= 5) {
                this.sudoForm = work[0].trim();
                this.type = work[1].trim();
                this.sudoFormAfter = work[2].trim();
                this.javaBefore = work[3].trim();
                this.javaAfter = work[4].trim();
            } else {
                System.err.println(
                        "WARN: Invalid type-based format in ConfigFile.txt line: " + importLine);
            }
        } else if (importLine.contains(
                "~~")) { // Options-based: sudo~javaBefore~javaAfter~opt1~opt2~~sudoOpt1~sudoOpt2
            String[] mainParts = importLine.split("~~", -1);
            if (mainParts.length >= 1) { // Must have at least the first part
                String[] work = mainParts[0].split("~", -1);
                if (work.length >= 3) {
                    this.sudoForm = work[0].trim();
                    this.javaBefore = work[1].trim();
                    this.javaAfter = work[2].trim();

                    // Populate javaOptions
                    for (int i = 3; i < work.length; i++) {
                        javaOptions.add(work[i].trim());
                    }
                } else {
                    System.err.println(
                            "WARN: Invalid options format (part 1, needs at least 3 '~' separated"
                                    + " values) in ConfigFile.txt line: "
                                    + importLine);
                }
                // Populate sudoOptions if the second part (after ~~) exists
                if (mainParts.length >= 2) {
                    String[] sudoOptsArr = mainParts[1].split("~", -1);
                    for (String opt : sudoOptsArr) {
                        sudoOptions.add(opt.trim());
                    }
                } else {
                    // If no ~~ part, it implies no specific sudoOptions, or an error in config line
                    // Depending on logic, might add a default empty sudoOption if javaOptions exist
                    if (!javaOptions.isEmpty() && sudoOptions.isEmpty()) {
                        // This case is a bit ambiguous based on original parsing.
                        // If javaOptions exist, there should usually be corresponding sudoOptions.
                        // System.err.println("WARN: Options format has javaOptions but no
                        // sudoOptions part (missing '~~') in ConfigFile.txt line: " + importLine);
                    }
                }
            } else {
                System.err.println(
                        "WARN: Invalid options format (empty or malformed '~~') in ConfigFile.txt"
                                + " line: "
                                + importLine);
            }
        } else {
            // Fallback or unrecognized simple command format
            // System.err.println("WARN: Unrecognized format in ConfigFile.txt line (treating as
            // simple command): " + importLine);
            this.sudoForm = importLine.trim(); // Assume the whole line is the sudoForm
            this.javaBefore = ""; // No specific Java transformation known
            this.javaAfter = "";
        }
    }

    @Override
    public String toString() {
        return "Callable{"
                + "sudoForm='"
                + sudoForm
                + '\''
                + ", javaBefore='"
                + javaBefore
                + '\''
                + ", javaAfter='"
                + javaAfter
                + '\''
                + ", javaOptions="
                + javaOptions
                + ", sudoOptions="
                + sudoOptions
                + ", typeBased="
                + typeBased
                + ", type='"
                + type
                + '\''
                + ", sudoFormAfter='"
                + sudoFormAfter
                + '\''
                + '}';
    }
}

// Inner class Line
class Line {
    public int indentation = 0;
    public String text; // Original text for this line, after leading tabs removed
    private final List<Callables> availableCallables;

    public Line(String rawText, List<Callables> callables) {
        this.availableCallables = callables;
        String currentText = rawText;
        while (currentText.startsWith("\t")) {
            currentText = currentText.substring(1);
            indentation++;
        }
        this.text = currentText; // Store the processed text
    }

    public String getJava() {
        // Prepare the text for matching: remove ALL internal whitespace for robust matching,
        // but keep the original `this.text` for error reporting.
        String matchText = this.text.replaceAll("\\s+", ""); // Collapses all whitespace

        if (matchText.isEmpty()) {
            return ""; // Return empty for effectively empty lines (e.g. lines with only
            // spaces/tabs)
        }

        for (Callables callable : availableCallables) {
            if (callable.sudoForm == null || callable.sudoForm.isEmpty()) {
                continue; // Skip improperly configured callables
            }

            String callableSudoFormProcessed = callable.sudoForm.replaceAll("\\s+", "");

            // Type-Based Matching
            if (callable.typeBased) {
                String callableSudoFormAfterProcessed =
                        (callable.sudoFormAfter != null)
                                ? callable.sudoFormAfter.replaceAll("\\s+", "")
                                : null;
                if (callableSudoFormAfterProcessed != null
                        && matchText.startsWith(callableSudoFormProcessed)
                        && matchText.endsWith(callableSudoFormAfterProcessed)) {

                    int startIndex = callableSudoFormProcessed.length();
                    int endIndex = matchText.length() - callableSudoFormAfterProcessed.length();

                    if (startIndex > endIndex)
                        continue; // Should not happen if startsWith and endsWith are true

                    String content =
                            matchText.substring(
                                    startIndex,
                                    endIndex); // This content is already whitespace-free

                    boolean valid = false;
                    if (callable.type == null) {
                        System.err.println(
                                "WARN: Callable type is null for type-based rule: "
                                        + callable.sudoForm);
                        continue;
                    }

                    if (callable.type.equals("double")) {
                        try {
                            Double.parseDouble(content);
                            valid = true;
                        } catch (NumberFormatException e) {
                        }
                    } else if (callable.type.equals("int")) {
                        try {
                            Integer.parseInt(content);
                            valid = true;
                        } catch (NumberFormatException e) {
                        }
                    } else { // Assume other types (e.g. "String") are valid if content exists
                        valid = !content.isEmpty();
                    }

                    if (valid) {
                        return callable.javaBefore
                                + content
                                + callable.javaAfter
                                + "\n"; // Add newline here
                    } else {
                        // System.err.println("WARN: Type mismatch for content '" + content + "'
                        // (rule: " + callable.sudoForm + ") on line: " + this.text);
                        // Don't return error yet, another rule might match better
                    }
                }
            }
            // Options-Based Matching
            else {
                if (matchText.startsWith(callableSudoFormProcessed)) {
                    String optionPart = matchText.substring(callableSudoFormProcessed.length());
                    // optionPart is already whitespace-free.
                    // callable.sudoOptions should also contain whitespace-free options if parsed
                    // correctly.

                    if (callable.sudoOptions != null) {
                        int optionIndex = -1;
                        // Iterate and check because sudoOptions in Callables might still have
                        // spaces if not trimmed during parsing
                        for (int k = 0; k < callable.sudoOptions.size(); k++) {
                            if (optionPart.equals(
                                    callable.sudoOptions.get(k).replaceAll("\\s+", ""))) {
                                optionIndex = k;
                                break;
                            }
                        }

                        if (optionIndex != -1
                                && callable.javaOptions != null
                                && optionIndex < callable.javaOptions.size()) {
                            String javaOption =
                                    callable.javaOptions.get(
                                            optionIndex); // This should be used as is
                            return callable.javaBefore
                                    + javaOption
                                    + callable.javaAfter
                                    + "\n"; // Add newline here
                        }
                    }
                }
            }
        }
        // If no rule matched after checking all callables
        System.err.println(
                "ERROR: No matching rule found for line: \""
                        + this.text
                        + "\" (Processed for match: \""
                        + matchText
                        + "\")");
        return "error";
    }
}
