import React, { useState, useEffect } from 'react';
import { Box, Typography, Paper, Divider } from '@mui/material';

interface BookPage {
  id: string;
  content: string;
  pageNumber: number;
  title?: string;
}

interface BookReaderProps {
  bookId: string;
  initialPage?: number;
  onContentSelect?: (selectedText: string) => void;
  onNavigateToReference?: (reference: any) => void;
  onReadingContextChange?: (context: ReadingContext) => void;
}

interface ReadingContext {
  page_number: number;
  current_section?: string;
  selected_text?: string;
}

const BookReader: React.FC<BookReaderProps> = ({
  bookId,
  initialPage = 1,
  onContentSelect,
  onNavigateToReference,
  onReadingContextChange
}) => {
  const [currentPage, setCurrentPage] = useState<number>(initialPage);
  const [bookContent, setBookContent] = useState<BookPage[]>([]);
  const [selectedText, setSelectedText] = useState<string>('');

  // Mock book content - in a real app this would come from the backend
  useEffect(() => {
    // In a real implementation, we would fetch book content based on bookId
    const mockPages: BookPage[] = [
      { id: 'page-1', content: 'This is the first page of the book. It contains introductory material and the beginning of chapter 1.', pageNumber: 1, title: 'Introduction' },
      { id: 'page-2', content: 'This is the second page. It continues chapter 1 with more details about the topic.', pageNumber: 2, title: 'Chapter 1, Continued' },
      { id: 'page-3', content: 'This is the third page. Chapter 1 is concluding and preparing to transition to the next section.', pageNumber: 3, title: 'Chapter 1, Conclusion' },
    ];
    setBookContent(mockPages);
  }, [bookId]);

  const handleTextSelection = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      const selectedText = selection.toString();
      setSelectedText(selectedText);
      if (onContentSelect) {
        onContentSelect(selectedText);
      }
    }
  };

  const handleQuerySelectedText = () => {
    // Trigger a query with the selected text
    // This would typically involve passing the selected text to the ChatInterface
    // along with a flag indicating it's a selected-text-only query
    if (selectedText && selectedText.trim() !== '') {
      // In a real implementation, this would trigger a special query mechanism
      // For now, we just log it
      console.log(`Querying about selected text: ${selectedText.substring(0, 50)}...`);
    }
  };

  const navigateToReference = (reference: any) => {
    if (onNavigateToReference) {
      onNavigateToReference(reference);
    }
  };

  const handlePageChange = (page: number) => {
    if (page >= 1 && page <= bookContent.length) {
      setCurrentPage(page);
      // Notify about reading context change
      if (onReadingContextChange) {
        const pageContent = bookContent.find(p => p.pageNumber === page);
        onReadingContextChange({
          page_number: page,
          current_section: pageContent ? pageContent.title || pageContent.id : undefined
        });
      }
    }
  };

  const currentPageContent = bookContent.find(page => page.pageNumber === currentPage)?.content || '';

  return (
    <Paper sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <Box sx={{ p: 2, borderBottom: 1, borderColor: 'divider' }}>
        <Typography variant="h6" component="h2">
          Book Reader
        </Typography>
        <Typography variant="subtitle2">
          Page {currentPage} of {bookContent.length}
        </Typography>
      </Box>
      
      <Box sx={{ flex: 1, p: 2, overflowY: 'auto' }}>
        <Box
          sx={{ 
            userSelect: 'text',
            cursor: 'text',
            minHeight: '400px',
            '&:hover': { backgroundColor: 'action.hover' },
            '&:focus': { backgroundColor: 'action.selected' }
          }}
          onMouseUp={handleTextSelection}
          onDoubleClick={() => window.getSelection()?.selectAllChildren(event.target as Node)}
        >
          <Typography variant="body1" paragraph>
            {currentPageContent}
          </Typography>
          
          {/* Additional pages would be rendered here in a real implementation */}
        </Box>
      </Box>
      
      <Box sx={{ p: 2, borderTop: 1, borderColor: 'divider', display: 'flex', justifyContent: 'space-between' }}>
        <button 
          onClick={() => handlePageChange(currentPage - 1)} 
          disabled={currentPage <= 1}
          style={{ padding: '8px 16px', cursor: currentPage <= 1 ? 'not-allowed' : 'pointer' }}
        >
          Previous
        </button>
        
        <Typography variant="body2">
          Page {currentPage}
        </Typography>
        
        <button 
          onClick={() => handlePageChange(currentPage + 1)} 
          disabled={currentPage >= (bookContent.length || 1)}
          style={{ padding: '8px 16px', cursor: currentPage >= (bookContent.length || 1) ? 'not-allowed' : 'pointer' }}
        >
          Next
        </button>
      </Box>
    </Paper>
  );
};

export default BookReader;